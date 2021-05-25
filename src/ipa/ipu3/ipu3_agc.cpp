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
#include <stdint.h>

#include <linux/v4l2-controls.h>

#include "libcamera/internal/log.h"

#include "libipa/histogram.h"

namespace libcamera {

namespace ipa::ipu3 {

LOG_DEFINE_CATEGORY(IPU3Agc)

/* Number of frames to wait before calculating stats on minimum exposure */
static constexpr uint32_t kInitialFrameMinAECount = 4;
/* Number of frames to wait between new gain/exposure estimations */
static constexpr uint32_t kFrameSkipCount = 6;

/* Histogram constants */
static constexpr uint32_t knumHistogramBins = 256;
static constexpr double kEvGainTarget = 0.25;

/* A cell is 8 bytes and contains averages for RGB values and saturation ratio */
static constexpr uint8_t kCellSize = 8;

/* seems to be a 13-bit pipeline */
static constexpr uint8_t kPipelineBits = 13;

IPU3Agc::IPU3Agc()
	: frameCount_(0), lastFrame_(0), converged_(false),
	  updateControls_(false), iqMean_(0.0), gamma_(1.0),
	  prevExposure_(0.0), prevExposureNoDg_(0.0),
	  currentExposure_(0.0), currentExposureNoDg_(0.0)
{
}

void IPU3Agc::initialise(struct ipu3_uapi_grid_config &bdsGrid, const IPAConfigInfo &configInfo)
{
	aeGrid_ = bdsGrid;
	ctrls_ = configInfo.entityControls.at(0);

	const auto itExp = ctrls_.find(V4L2_CID_EXPOSURE);
	if (itExp == ctrls_.end()) {
		LOG(IPU3Agc, Error) << "Can't find exposure control";
		return;
	}
	minExposure_ = itExp->second.min().get<int32_t>();
	maxExposure_ = itExp->second.max().get<int32_t>();
	lineDuration_ = configInfo.sensorInfo.lineLength / (configInfo.sensorInfo.pixelRate / 1e6);
	maxExposureTime_ = maxExposure_ * lineDuration_;

	const auto itGain = ctrls_.find(V4L2_CID_ANALOGUE_GAIN);
	if (itGain == ctrls_.end()) {
		LOG(IPU3Agc, Error) << "Can't find gain control";
		return;
	}
	minGain_ = std::max(itGain->second.min().get<int32_t>(), 1);
	maxGain_ = 15;//itGain->second.max().get<int32_t>();
}

/* Generate an RGB vector with the average values for each region */
void IPU3Agc::generateZones(std::vector<RGB> &zones)
{
	for (unsigned int i = 0; i < kAwbStatsSizeX * kAwbStatsSizeY; i++) {
		RGB zone;
		double counted = awbStats_[i].counted;
		LOG(IPU3Agc, Debug) << counted << " counted values with a minimum of "
				    << minZonesCounted_ << " "
				    << awbStats_[i].gSum / counted << " green zones";
		if (counted >= minZonesCounted_) {
			zone.G = awbStats_[i].gSum / counted;
			if (zone.G >= kMinGreenLevelInZone) {
				zone.R = awbStats_[i].rSum / counted;
				zone.B = awbStats_[i].bSum / counted;
				zones.push_back(zone);
			}
		}
	}
}

/* Translate the IPU3 statistics into the default statistics region array */
void IPU3Agc::generateStats(const ipu3_uapi_stats_3a *stats)
{
	uint32_t regionWidth = round(aeGrid_.width / static_cast<double>(kAwbStatsSizeX));
	uint32_t regionHeight = round(aeGrid_.height / static_cast<double>(kAwbStatsSizeY));

	minZonesCounted_ = ((regionWidth * regionHeight) * 4) / 5;
	/*
	 * Generate a (kAwbStatsSizeX x kAwbStatsSizeY) array from the IPU3 grid which is
	 * (aeGrid_.width x aeGrid_.height).
	 */
	for (unsigned int j = 0; j < kAwbStatsSizeY * regionHeight; j++) {
		for (unsigned int i = 0; i < kAwbStatsSizeX * regionWidth; i++) {
			uint32_t cellPosition = j * aeGrid_.width + i;
			uint32_t cellX = (cellPosition / regionWidth) % kAwbStatsSizeX;
			uint32_t cellY = ((cellPosition / aeGrid_.width) / regionHeight) % kAwbStatsSizeY;

			uint32_t awbRegionPosition = cellY * kAwbStatsSizeX + cellX;
			cellPosition *= 8;

			/* Cast the initial IPU3 structure to simplify the reading */
			Ipu3AwbCell *currentCell = reinterpret_cast<Ipu3AwbCell *>(const_cast<uint8_t *>(&stats->awb_raw_buffer.meta_data[cellPosition]));
			if (currentCell->satRatio == 0) {
				/* The cell is not saturated, use the current cell */
				awbStats_[awbRegionPosition].counted++;
				uint32_t greenValue = currentCell->greenRedAvg + currentCell->greenBlueAvg;
				awbStats_[awbRegionPosition].gSum += greenValue / 2;
				awbStats_[awbRegionPosition].rSum += currentCell->redAvg;
				awbStats_[awbRegionPosition].bSum += currentCell->blueAvg;
			}
		}
	}
}

void IPU3Agc::clearStats()
{
	for (unsigned int i = 0; i < kAwbStatsSizeX * kAwbStatsSizeY; i++) {
		awbStats_[i].bSum = 0;
		awbStats_[i].rSum = 0;
		awbStats_[i].gSum = 0;
		awbStats_[i].counted = 0;
		awbStats_[i].uncounted = 0;
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
	int topleftX = topleft.x >> aeGrid_.block_width_log2;
	int topleftY = topleft.y >> aeGrid_.block_height_log2;

	/* Align to the grid cell width and height */
	uint32_t startX = topleftX << aeGrid_.block_width_log2;
	uint32_t startY = topleftY * aeGrid_.width << aeGrid_.block_width_log2;
	uint32_t endX = (startX + (aeRegion.size().width >> aeGrid_.block_width_log2)) << aeGrid_.block_width_log2;
	uint32_t i, j;
	uint32_t count = 0;

	uint32_t hist[knumHistogramBins] = { 0 };
	for (j = topleftY;
	     j < topleftY + (aeRegion.size().height >> aeGrid_.block_height_log2);
	     j++) {
		for (i = startX + startY; i < endX + startY; i += kCellSize) {
			/*
			 * The grid width (and maybe height) is not reliable.
			 * We observed a bit shift which makes the value 160 to be 32 in the stats grid.
			 * Use the one passed at init time.
			 */
			if (stats->awb_raw_buffer.meta_data[i + 4 + j * aeGrid_.width] == 0) {
				uint8_t Gr = stats->awb_raw_buffer.meta_data[i + 0 + j * aeGrid_.width];
				uint8_t Gb = stats->awb_raw_buffer.meta_data[i + 3 + j * aeGrid_.width];
				hist[(Gr + Gb) / 2]++;
				count++;
			}
		}
	}

	/* Limit the gamma effect for now */
	gamma_ = 2.2;

	/* Estimate the quantile mean of the top 2% of the histogram */
	iqMean_ = Histogram(Span<uint32_t>(hist)).interQuantileMean(0.98, 1.0);
}

void IPU3Agc::filterExposure()
{
	double speed = 0.1;
	if (prevExposure_ == 0.0) {
		/* DG stands for digital gain.*/
		prevExposure_ = currentExposure_;
		prevExposureNoDg_ = currentExposureNoDg_;
	} else {
		/*
		 * If we are close to the desired result, go faster to avoid making
		 * multiple micro-adjustments.
		 * \ todo: Make this customisable?
		 */
		if (prevExposure_ < 1.2 * currentExposure_ &&
		    prevExposure_ > 0.8 * currentExposure_)
			speed = sqrt(speed);

		prevExposure_ = speed * currentExposure_ +
				prevExposure_ * (1.0 - speed);
		prevExposureNoDg_ = speed * currentExposureNoDg_ +
				prevExposureNoDg_ * (1.0 - speed);
	}
	/*
	 * We can't let the no_dg exposure deviate too far below the
	 * total exposure, as there might not be enough digital gain available
	 * in the ISP to hide it (which will cause nasty oscillation).
	 */
	double fastReduceThreshold = 0.3;
	if (prevExposureNoDg_ <
	    prevExposure_ * fastReduceThreshold)
		prevExposureNoDg_ = prevExposure_ * fastReduceThreshold;
	LOG(IPU3Agc, Debug) << "After filtering, total_exposure " << prevExposure_;
}

void IPU3Agc::lockExposureGain(uint32_t &exposure, uint32_t &gain)
{
	updateControls_ = false;

	/* Algorithm initialization should wait for first valid frames */
	/* \todo - have a number of frames given by DelayedControls ?
	 * - implement a function for IIR */
	if ((frameCount_ < kInitialFrameMinAECount) || (frameCount_ - lastFrame_ < kFrameSkipCount))
		return;

	/* Are we correctly exposed ? */
	if (std::abs(iqMean_ - kEvGainTarget * knumHistogramBins) <= 1) {
		LOG(IPU3Agc, Debug) << "!!! Good exposure with iqMean = " << iqMean_;
		converged_ = true;
	} else {
		double newGain = kEvGainTarget * knumHistogramBins / iqMean_;

		/* extracted from Rpi::Agc::computeTargetExposure */
		double currentShutter = exposure * lineDuration_;
		currentExposureNoDg_ = currentShutter * gain;
		LOG(IPU3Agc, Debug) << "Actual total exposure " << currentExposureNoDg_
				    << " Shutter speed " << currentShutter
				    << " Gain " << gain;
		currentExposure_ = currentExposureNoDg_ * newGain;
		double maxTotalExposure = maxExposureTime_ * maxGain_;
		currentExposure_ = std::min(currentExposure_, maxTotalExposure);
		LOG(IPU3Agc, Debug) << "Target total exposure " << currentExposure_;

		/* \todo: estimate if we need to desaturate */
		filterExposure();

		double newExposure = 0.0;
		if (currentShutter < maxExposureTime_) {
			exposure = std::clamp(static_cast<uint32_t>(exposure * currentExposure_ / currentExposureNoDg_), minExposure_, maxExposure_);
			newExposure = currentExposure_ / exposure;
			gain = std::clamp(static_cast<uint32_t>(gain * currentExposure_ / newExposure), minGain_, maxGain_);
			updateControls_ = true;
		} else if (currentShutter >= maxExposureTime_) {
			gain = std::clamp(static_cast<uint32_t>(gain * currentExposure_ / currentExposureNoDg_), minGain_, maxGain_);
			newExposure = currentExposure_ / gain;
			exposure = std::clamp(static_cast<uint32_t>(exposure * currentExposure_ / newExposure), minExposure_, maxExposure_);
			updateControls_ = true;
		}
		LOG(IPU3Agc, Debug) << "Adjust exposure " << exposure * lineDuration_ << " and gain " << gain;
	}
	lastFrame_ = frameCount_;
}

double IPU3Agc::compute_initial_Y(IspStatsRegion regions[], AwbStatus const &awb,
				double weights[], double gain)
{
	// Note how the calculation below means that equal weights give you
	// "average" metering (i.e. all pixels equally important).
	double R_sum = 0, G_sum = 0, B_sum = 0, pixel_sum = 0;
	for (unsigned int i = 0; i < kAwbStatsSizeX * kAwbStatsSizeY; i++) {
		double counted = regions[i].counted;
		double r_sum = std::min(regions[i].rSum * gain, ((1 << kPipelineBits) - 1) * counted);
		double g_sum = std::min(regions[i].gSum * gain, ((1 << kPipelineBits) - 1) * counted);
		double b_sum = std::min(regions[i].bSum * gain, ((1 << kPipelineBits) - 1) * counted);
		R_sum += r_sum * weights[i];
		G_sum += g_sum * weights[i];
		B_sum += b_sum * weights[i];
		pixel_sum += counted * weights[i];
	}
	if (pixel_sum == 0.0) {
		LOG(IPU3Agc, Warning) << "compute_initial_Y: pixel_sum is zero";
		return 0;
	}
	double Y_sum = R_sum * awb.redGain * .299 +
		       G_sum * awb.greenGain * .587 +
		       B_sum * awb.blueGain * .114;
	return Y_sum / pixel_sum / (1 << kPipelineBits);
}

void IPU3Agc::process(const ipu3_uapi_stats_3a *stats, uint32_t &exposure, uint32_t &gain)
{
	ASSERT(stats->stats_3a_status.awb_en);
	zones_.clear();
	clearStats();
	generateStats(stats);
	generateZones(zones_);
	
	lockExposureGain(exposure, gain);
	frameCount_++;
}

} /* namespace ipa::ipu3 */

} /* namespace libcamera */
