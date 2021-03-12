/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Ideas On Board
 *
 * ipu3_agc.cpp - AGC/AEC control algorithm
 */

#include "ipu3_agc.h"

#include <algorithm>
#include <cmath>
#include <numeric>

#include "libcamera/internal/log.h"

#include "libipa/histogram.h"

namespace libcamera {

namespace ipa {

LOG_DEFINE_CATEGORY(IPU3Agc)

/* Number of frames to wait before calculating stats on minimum exposure */
static const uint32_t kInitialFrameMinAECount = 4;
/* Number of frames to wait between new gain/exposure estimations */
static const uint32_t kFrameSkipCount = 6;

/* Maximum ISO value for analogue gain */
static const uint32_t kMinISO = 100;
static const uint32_t kMaxISO = 1500;
/* Maximum analogue gain value
 * \todo grab it from a camera helper */
static const uint32_t kMinGain = kMinISO / 100;
static const uint32_t kMaxGain = kMaxISO / 100;
/* \todo use calculated value based on sensor */
static const uint32_t kMinExposure = 1;
static const uint32_t kMaxExposure = 1976;
/* \todo those should be get from pipeline handler ! */
/* line duration in microseconds */
static const double kLineDuration = 16.8;
static const double kMaxExposureTime = kMaxExposure * kLineDuration;
/* Histogram constants */
static const uint32_t knumHistogramBins = 256;
static const double kEvGainTarget = 0.5;

IPU3Agc::IPU3Agc()
	: frameCount_(0), lastFrame_(0),
	  converged_(false), updateControls_(false)
{
	iqMean_ = 0.0;
	gamma_ = 1.0;
	histLow_ = 0;
	histHigh_ = 255;
	prevTotalExposure_ = 0.0;
	prevTotalExposureNoDg_ = 0.0;
	currentTotalExposure_ = 0.0;
	currentTotalExposureNoDg_ = 0.0;
}

IPU3Agc::~IPU3Agc()
{
}

void IPU3Agc::initialise(struct ipu3_uapi_grid_config &bdsGrid)
{
	aeGrid_ = bdsGrid;
	cellsBrightness_.reserve(IPU3_UAPI_AWB_MAX_BUFFER_SIZE);
}
void IPU3Agc::processBrightness(const ipu3_uapi_stats_3a *stats)
{
	const struct ipu3_uapi_grid_config statsAeGrid = stats->stats_4a_config.awb_config.grid;
	Rectangle aeRegion = { statsAeGrid.x_start,
			       statsAeGrid.y_start,
			       static_cast<unsigned int>(statsAeGrid.x_end - statsAeGrid.x_start) + 1,
			       static_cast<unsigned int>(statsAeGrid.y_end - statsAeGrid.y_start) + 1 };
	Point topleft = aeRegion.topLeft();
	uint32_t startY = (topleft.y >> aeGrid_.block_height_log2) * aeGrid_.width << aeGrid_.block_width_log2;
	uint32_t startX = (topleft.x >> aeGrid_.block_width_log2) << aeGrid_.block_width_log2;
	uint32_t endX = (startX + (aeRegion.size().width >> aeGrid_.block_width_log2)) << aeGrid_.block_width_log2;
	uint32_t i, j;
	uint32_t count = 0;

	cellsBrightness_.clear();

	for (j = (topleft.y >> aeGrid_.block_height_log2);
	     j < (topleft.y >> aeGrid_.block_height_log2) + (aeRegion.size().height >> aeGrid_.block_height_log2);
	     j++) {
		for (i = startX + startY; i < endX + startY; i += 8) {
			/* grid width (and maybe height) is not reliable.
			 * We observed a bit shift which makes the value 160 to be 32 in the stats grid.
			 * Use the one passed at init time. */
			if (stats->awb_raw_buffer.meta_data[i + 4 + j * aeGrid_.width] == 0) {
				uint8_t Gr = stats->awb_raw_buffer.meta_data[i + j * aeGrid_.width];
				uint8_t R = stats->awb_raw_buffer.meta_data[i + 1 + j * aeGrid_.width];
				uint8_t B = stats->awb_raw_buffer.meta_data[i + 2 + j * aeGrid_.width];
				uint8_t Gb = stats->awb_raw_buffer.meta_data[i + 3 + j * aeGrid_.width];

				cellsBrightness_.push_back(static_cast<uint32_t>(0.2125 * R + 0.7154 * (Gr + Gb) / 2 + 0.0722 * B));
				count++;
			}
		}
	}
	std::vector<uint32_t>::iterator maxIntensity = std::max_element(cellsBrightness_.begin(), cellsBrightness_.end());
	LOG(IPU3Agc, Debug) << "Most frequent intensity is " << *maxIntensity << " at " << std::distance(cellsBrightness_.begin(), maxIntensity);

	/* \todo create a class to generate histograms ! */
	uint32_t hist[knumHistogramBins] = { 0 };
	for (uint32_t const &val : cellsBrightness_)
		hist[val]++;

	double mean = 0.0;
	for (i = 0; i < knumHistogramBins; i++) {
		mean += hist[i] * i;
	}
	mean /= count;

	double variance = 0.0;
	for (i = 0; i < knumHistogramBins; i++) {
		variance += ((i - mean) * (i - mean)) * hist[i];
	}
	variance /= count;
	variance = std::sqrt(variance);

	LOG(IPU3Agc, Debug) << "mean value is: " << mean << " and variance is " << variance;
	/* Limit the gamma effect for now */
	gamma_ = 1.4;

	const auto [minBrightness, maxBrightness] = std::minmax_element(cellsBrightness_.begin(), cellsBrightness_.end());
	histLow_ = *minBrightness;
	histHigh_ = *maxBrightness;

	iqMean_ = Histogram(Span<uint32_t>(hist)).interQuantileMean(0.98, 1.0);
}

void IPU3Agc::filterExposure(bool desaturate)
{
	double speed = 0.2;
	if (prevTotalExposure_ == 0.0) {
		prevTotalExposure_ = currentTotalExposure_;
		prevTotalExposureNoDg_ = currentTotalExposureNoDg_;
	} else {
		/* If close to the result go faster, to save making so many
		 * micro-adjustments on the way.
		 * \ todo: Make this customisable? */
		if (prevTotalExposure_ < 1.2 * currentTotalExposure_ &&
		    prevTotalExposure_ > 0.8 * currentTotalExposure_)
			speed = sqrt(speed);
		prevTotalExposure_ = speed * currentTotalExposure_ +
				     prevTotalExposure_ * (1.0 - speed);
		/* When desaturing, take a big jump down in exposure_no_dg,
		 * which we'll hide with digital gain. */
		if (desaturate)
			prevTotalExposureNoDg_ =
				currentTotalExposureNoDg_;
		else
			prevTotalExposureNoDg_ =
				speed * currentTotalExposureNoDg_ +
				prevTotalExposureNoDg_ * (1.0 - speed);
	}
	/* We can't let the no_dg exposure deviate too far below the
	 * total exposure, as there might not be enough digital gain available
	 * in the ISP to hide it (which will cause nasty oscillation). */
	double fastReduceThreshold = 0.4;
	if (prevTotalExposureNoDg_ <
	    prevTotalExposure_ * fastReduceThreshold)
		prevTotalExposureNoDg_ = prevTotalExposure_ * fastReduceThreshold;
	LOG(IPU3Agc, Debug) << "After filtering, total_exposure " << prevTotalExposure_;
}

void IPU3Agc::lockExposureGain(uint32_t &exposure, uint32_t &gain)
{
	updateControls_ = false;

	/* Algorithm initialization wait for first valid frames */
	/* \todo - have a number of frames given by DelayedControls ?
	 * - implement a function for IIR */
	if ((frameCount_ == kInitialFrameMinAECount) || (frameCount_ - lastFrame_ >= kFrameSkipCount)) {
		/* Are we correctly exposed ? */
		double newGain = kEvGainTarget * knumHistogramBins / iqMean_;

		if (std::abs(iqMean_ - kEvGainTarget * knumHistogramBins) <= 1) {
			LOG(IPU3Agc, Debug) << "!!! Good exposure with iqMean = " << iqMean_;
			converged_ = true;
		} else {
			/* extracted from Rpi::Agc::computeTargetExposure */
			double currentShutter = exposure * kLineDuration;
			currentTotalExposureNoDg_ = currentShutter * gain;
			LOG(IPU3Agc, Debug) << "Actual total exposure " << currentTotalExposureNoDg_
					    << " Shutter speed " << currentShutter
					    << " Gain " << gain;
			currentTotalExposure_ = currentTotalExposureNoDg_ * newGain;
			double maxTotalExposure = kMaxExposureTime * kMaxGain;
			currentTotalExposure_ = std::min(currentTotalExposure_, maxTotalExposure);
			LOG(IPU3Agc, Debug) << "Target total exposure " << currentTotalExposure_;

			/* \todo: estimate if we need to desaturate */
			filterExposure(false);

			double newExposure = 0.0;
			if (currentShutter < kMaxExposureTime) {
				exposure = std::clamp(static_cast<uint32_t>(exposure * currentTotalExposure_ / currentTotalExposureNoDg_), kMinExposure, kMaxExposure);
				newExposure = currentTotalExposure_ / exposure;
				gain = std::clamp(static_cast<uint32_t>(gain * currentTotalExposure_ / newExposure), kMinGain, kMaxGain);
				updateControls_ = true;
			} else if (currentShutter >= kMaxExposureTime) {
				gain = std::clamp(static_cast<uint32_t>(gain * currentTotalExposure_ / currentTotalExposureNoDg_), kMinGain, kMaxGain);
				newExposure = currentTotalExposure_ / gain;
				exposure = std::clamp(static_cast<uint32_t>(exposure * currentTotalExposure_ / newExposure), kMinExposure, kMaxExposure);
				updateControls_ = true;
			}
			LOG(IPU3Agc, Debug) << "Adjust exposure " << exposure * kLineDuration << " and gain " << gain;
		}
		lastFrame_ = frameCount_;
	} else {
		updateControls_ = false;
	}
}

void IPU3Agc::process(const ipu3_uapi_stats_3a *stats, uint32_t &exposure, uint32_t &gain)
{
	processBrightness(stats);
	lockExposureGain(exposure, gain);
	frameCount_++;
}

} /* namespace ipa */

} /* namespace libcamera */
