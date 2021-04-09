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
static constexpr uint32_t kInitialFrameMinAECount = 4;
/* Number of frames to wait between new gain/exposure estimations */
static constexpr uint32_t kFrameSkipCount = 6;

/* Maximum ISO value for analogue gain */
static constexpr uint32_t kMinISO = 100;
static constexpr uint32_t kMaxISO = 1500;
/* Maximum analogue gain value
 * \todo grab it from a camera helper */
static constexpr uint32_t kMinGain = kMinISO / 100;
static constexpr uint32_t kMaxGain = kMaxISO / 100;
/* \todo use calculated value based on sensor */
static constexpr uint32_t kMinExposure = 1;
static constexpr uint32_t kMaxExposure = 1976;
/* \todo those should be get from pipeline handler ! */
/* line duration in microseconds */
static constexpr double kLineDuration = 16.8;
static constexpr double kMaxExposureTime = kMaxExposure * kLineDuration;
/* Histogram constants */
static constexpr uint32_t knumHistogramBins = 256;
static constexpr double kEvGainTarget = 0.5;

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
}

double IPU3Agc::compute_initial_Y(double weights[], double gain)
{
	ispStatsRegion *regions = agcStats_;
	// Note how the calculation below means that equal weights give you
	// "average" metering (i.e. all pixels equally important).
	double redSum = 0, greenSum = 0, blueSum = 0, pixelSum = 0;
	for (unsigned int i = 0; i < kAgcStatsSizeX * kAgcStatsSizeY; i++) {
		double counted = regions[i].counted;
		double rSum = std::min(regions[i].rSum * gain, ((1 << 8) - 1) * counted);
		double gSum = std::min(regions[i].gSum * gain, ((1 << 8) - 1) * counted);
		double bSum = std::min(regions[i].bSum * gain, ((1 << 8) - 1) * counted);
		redSum += rSum * weights[i];
		greenSum += gSum * weights[i];
		blueSum += bSum * weights[i];
		pixelSum += counted * weights[i];
	}
	if (pixelSum == 0.0) {
		LOG(IPU3Agc, Warning) << "compute_initial_Y: pixel_sum is zero";
		return 0;
	}
	double Y_sum = redSum * 1.68 * .299 +
		       greenSum * 1.0 * .587 +
		       blueSum * 1.72 * .114;
	return Y_sum / pixelSum / (1 << 8);
}

void IPU3Agc::generateAgcStats(const ipu3_uapi_stats_3a *stats)
{
	uint32_t regionWidth = round(aeGrid_.width / static_cast<double>(kAgcStatsSizeX));
	uint32_t regionHeight = round(aeGrid_.height / static_cast<double>(kAgcStatsSizeY));

	for (unsigned int j = 0; j < kAgcStatsSizeY * regionHeight; j++) {
		for (unsigned int i = 0; i < kAgcStatsSizeX * regionWidth; i++) {
			uint32_t cellPosition = j * aeGrid_.width + i;
			uint32_t cellX = (cellPosition / regionWidth) % kAgcStatsSizeX;
			uint32_t cellY = ((cellPosition / aeGrid_.width) / regionHeight) % kAgcStatsSizeY;

			uint32_t awbRegionPosition = cellY * kAgcStatsSizeX + cellX;
			cellPosition *= 8;
			if (stats->awb_raw_buffer.meta_data[cellPosition + 4] == 0) {
				/* The cell is not saturated */
				agcStats_[awbRegionPosition].counted++;
				uint32_t greenValue = stats->awb_raw_buffer.meta_data[cellPosition + 0] + stats->awb_raw_buffer.meta_data[cellPosition + 3];
				agcStats_[awbRegionPosition].gSum += greenValue / 2;
				agcStats_[awbRegionPosition].rSum += stats->awb_raw_buffer.meta_data[cellPosition + 1];
				agcStats_[awbRegionPosition].bSum += stats->awb_raw_buffer.meta_data[cellPosition + 2];
			}
		}
	}
}

void IPU3Agc::clearAgcStats()
{
	for (unsigned int i = 0; i < kAgcStatsSizeX * kAgcStatsSizeY; i++) {
		agcStats_[i].bSum = 0;
		agcStats_[i].rSum = 0;
		agcStats_[i].gSum = 0;
		agcStats_[i].counted = 0;
		agcStats_[i].notcounted = 0;
	}
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

	uint32_t brightness = 0;
	uint32_t hist[knumHistogramBins] = { 0 };

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

				brightness = 0.299 * R + 0.587 * (Gr + Gb) / 2 + 0.114 * B;
				hist[(Gr + Gb) / 2]++;
				count++;
			}
		}
	}

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
	clearAgcStats();
	generateAgcStats(stats);

	double mygain = 1.0;
	double weights[16] = { 1, 1, 1, 1, 1, 2, 2, 1, 1, 2, 2, 1, 1, 1, 1, 1 };
	double targetY = 0.9;

	for (int i = 0; i < 8; i++) {
		double initial_Y = compute_initial_Y(weights, mygain);
		double extra_gain = std::min(10.0, targetY / (initial_Y + .001));
		mygain *= extra_gain;
		LOG(IPU3Agc, Debug) << "Initial Y " << initial_Y << " target " << targetY
				    << " gives gain " << mygain;
	}

	processBrightness(stats);
	lockExposureGain(exposure, gain);
	frameCount_++;
}

} /* namespace ipa */

} /* namespace libcamera */
