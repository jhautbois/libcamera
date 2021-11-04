/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Ideas On Board
 *
 * ipu3_agc.cpp - AGC/AEC mean-based control algorithm
 */

#include "agc.h"

#include <algorithm>
#include <chrono>
#include <cmath>

#include <libcamera/base/log.h>

#include <libcamera/ipa/core_ipa_interface.h>

#include "libipa/histogram.h"

/**
 * \file agc.h
 */

namespace libcamera {

using namespace std::literals::chrono_literals;

namespace ipa::ipu3::algorithms {

/**
 * \class Agc
 * \brief A mean-based auto-exposure algorithm
 *
 * This algorithm calculates a shutter time and an analogue gain so that the
 * average value of the green channel of the brightest 2% of pixels approaches
 * 0.5. The AWB gains are not used here, and all cells in the grid have the same
 * weight, like an average-metering case. In this metering mode, the camera uses
 * light information from the entire scene and creates an average for the final
 * exposure setting, giving no weighting to any particular portion of the
 * metered area.
 *
 * Reference: Battiato, Messina & Castorina. (2008). Exposure
 * Correction for Imaging Devices: An Overview. 10.1201/9781420054538.ch12.
 */

LOG_DEFINE_CATEGORY(IPU3Agc)

/* Limits for analogue gain values */
static constexpr double kMinAnalogueGain = 1.0;
static constexpr double kMaxAnalogueGain = 8.0;

/* \todo Honour the FrameDurationLimits control instead of hardcoding a limit */
static constexpr utils::Duration kMaxShutterSpeed = 60ms;

/* Histogram constants */
static constexpr uint32_t knumHistogramBins = 256;

/* Target value to reach for the top 2% of the histogram */
static constexpr double kEvGainTarget = 0.5;

/*
 * Maximum ratio of saturated pixels in a cell for the cell to be considered
 * non-saturated and counted by the AGC algorithm.
 */
static constexpr uint32_t kMinCellsPerZoneRatio = 255 * 20 / 100;

/* Number of frames to wait before calculating stats on minimum exposure */
static constexpr uint32_t kNumStartupFrames = 10;

Agc::Agc()
	: frameCount_(0), iqMean_(0.0), lineDuration_(0s), minShutterSpeed_(0s),
	  maxShutterSpeed_(0s), filteredExposure_(0s), currentExposure_(0s),
	  prevExposureValue_(0s)
{
}

/**
 * \brief Configure the AGC given a configInfo
 * \param[in] context The shared IPA context
 * \param[in] configInfo The IPA configuration data
 *
 * \return 0
 */
int Agc::configure(IPAContext &context, const IPAConfigInfo &configInfo)
{
	stride_ = context.configuration.grid.stride;

	/* \todo use the IPAContext to provide the limits */
	lineDuration_ = configInfo.sensorInfo.lineLength * 1.0s
		      / configInfo.sensorInfo.pixelRate;

	minShutterSpeed_ = context.configuration.agc.minShutterSpeed;
	maxShutterSpeed_ = std::min(context.configuration.agc.maxShutterSpeed,
				    kMaxShutterSpeed);

	minAnalogueGain_ = std::max(context.configuration.agc.minAnalogueGain, kMinAnalogueGain);
	maxAnalogueGain_ = std::min(context.configuration.agc.maxAnalogueGain, kMaxAnalogueGain);

	/* Configure the default exposure and gain. */
	context.frameContext.agc.gain = minAnalogueGain_;
	context.frameContext.agc.exposure = minShutterSpeed_ / lineDuration_;

	prevExposureValue_ = context.frameContext.agc.gain
			   * context.frameContext.agc.exposure
			   * lineDuration_;

	return 0;
}

/**
 * \brief Estimate the mean value of the top 2% of the histogram
 * \param[in] stats The statistics computed by the ImgU
 * \param[in] grid The grid used to store the statistics in the IPU3
 */
void Agc::measureBrightness(const ipu3_uapi_stats_3a *stats,
			    const ipu3_uapi_grid_config &grid)
{
	/* Initialise the histogram array */
	uint32_t hist[knumHistogramBins] = { 0 };

	for (unsigned int cellY = 0; cellY < grid.height; cellY++) {
		for (unsigned int cellX = 0; cellX < grid.width; cellX++) {
			uint32_t cellPosition = cellY * stride_ + cellX;

			const ipu3_uapi_awb_set_item *cell =
				reinterpret_cast<const ipu3_uapi_awb_set_item *>(
					&stats->awb_raw_buffer.meta_data[cellPosition]
				);

			if (cell->sat_ratio <= kMinCellsPerZoneRatio) {
				uint8_t gr = cell->Gr_avg;
				uint8_t gb = cell->Gb_avg;
				/*
				 * Store the average green value to estimate the
				 * brightness. Even the overexposed pixels are
				 * taken into account.
				 */
				hist[(gr + gb) / 2]++;
			}
		}
	}

	Histogram cumulativeHist = Histogram(Span<uint32_t>(hist));
	/* Estimate the quantile mean of the top 2% of the histogram */
	if (cumulativeHist.total() == 0) {
		/* Force the value as histogram is empty */
		iqMean_ = knumHistogramBins - 0.5;
	} else {
		iqMean_ = cumulativeHist.interQuantileMean(0.98, 1.0);
	}
}

/**
 * \brief Apply a filter on the exposure value to limit the speed of changes
 */
void Agc::filterExposure()
{
	double speed = 0.2;

	/* Adapt instantly if we are in startup phase */
	if (frameCount_ < kNumStartupFrames)
		speed = 1.0;

	if (filteredExposure_ == 0s) {
		/* DG stands for digital gain.*/
		filteredExposure_ = currentExposure_;
	} else {
		/*
		 * If we are close to the desired result, go faster to avoid making
		 * multiple micro-adjustments.
		 * \todo Make this customisable?
		 */
		if (filteredExposure_ < 1.2 * currentExposure_ &&
		    filteredExposure_ > 0.8 * currentExposure_)
			speed = sqrt(speed);

		filteredExposure_ = speed * currentExposure_ +
				filteredExposure_ * (1.0 - speed);
	}

	LOG(IPU3Agc, Debug) << "After filtering, total_exposure " << filteredExposure_;
}

/**
 * \brief Estimate the new exposure and gain values
 * \param[inout] exposure The exposure value reference as a number of lines
 * \param[inout] gain The gain reference to be updated
 * \param[in] currentYGain The gain calculated on the current brightness level
 */
void Agc::computeExposure(uint32_t &exposure, double &analogueGain, double currentYGain)
{
	/* Estimate the gain needed to have the proportion wanted */
	double evGain = kEvGainTarget * knumHistogramBins / iqMean_;

	/* extracted from Rpi::Agc::computeTargetExposure */
	/* Calculate the shutter time in seconds */
	utils::Duration currentShutter = exposure * lineDuration_;
	/* Update the exposure value for the next computation. */
	prevExposureValue_ = currentShutter * analogueGain;

	LOG(IPU3Agc, Debug) << "Actual total exposure " << currentShutter * analogueGain
			    << " Shutter speed " << currentShutter
			    << " Gain " << analogueGain
			    << " Needed ev gain " << evGain;

	if (evGain < currentYGain)
		evGain = currentYGain;

	if (std::abs(evGain - 1.0) < 0.01) {
		LOG(IPU3Agc, Debug) << "We are well exposed (iqMean = "
				    << iqMean_ << ")";
		return;
	}

	/*
	 * Calculate the current exposure value for the scene as the latest
	 * exposure value applied multiplied by the new estimated gain.
	 */
	currentExposure_ = prevExposureValue_ * evGain;

	/* Clamp the exposure value to the min and max authorized */
	utils::Duration maxTotalExposure = maxShutterSpeed_ * maxAnalogueGain_;
	currentExposure_ = std::min(currentExposure_, maxTotalExposure);
	LOG(IPU3Agc, Debug) << "Target total exposure " << currentExposure_
			    << ", maximum is " << maxTotalExposure;

	/* \todo: estimate if we need to desaturate */
	filterExposure();

	/* Divide the exposure value as new exposure and gain values */
	utils::Duration exposureValue = filteredExposure_;
	utils::Duration shutterTime = minShutterSpeed_;

	/*
	* Push the shutter time up to the maximum first, and only then
	* increase the gain.
	*/
	shutterTime = std::clamp<utils::Duration>(exposureValue / minAnalogueGain_,
						  minShutterSpeed_, maxShutterSpeed_);
	double stepGain = std::clamp(exposureValue / shutterTime,
				     minAnalogueGain_, maxAnalogueGain_);
	LOG(IPU3Agc, Debug) << "Divided up shutter and gain are "
			    << shutterTime << " and "
			    << stepGain;

	exposure = shutterTime / lineDuration_;
	analogueGain = stepGain;
}

/**
 * \brief Estimate the average brightness of the frame
 * \param[in] context The shared IPA context
 * \param[in] grid The grid used to store the statistics in the IPU3
 * \param[in] stats The IPU3 statistics and ISP results
 * \param[in] currentYGain The gain calculated on the current brightness level
 */
double Agc::computeInitialY(IPAFrameContext &frameContext,
			    const ipu3_uapi_grid_config &grid,
			    const ipu3_uapi_stats_3a *stats,
			    double currentYGain)
{
	double redSum = 0, greenSum = 0, blueSum = 0;

	for (unsigned int cellY = 0; cellY < grid.height; cellY++) {
		for (unsigned int cellX = 0; cellX < grid.width; cellX++) {
			uint32_t cellPosition = cellY * stride_ + cellX;

			const ipu3_uapi_awb_set_item *cell =
				reinterpret_cast<const ipu3_uapi_awb_set_item *>(
					&stats->awb_raw_buffer.meta_data[cellPosition]
				);

			redSum += cell->R_avg * currentYGain;
			greenSum += (cell->Gr_avg + cell->Gb_avg) / 2 * currentYGain;
			blueSum += cell->B_avg * currentYGain;
		}
	}

	/*
	 * Estimate the sum of the brightness values, weighted with the gains
	 * applied on the channels in AWB.
	 */
	double Y_sum = redSum * frameContext.awb.gains.red * .299 +
		       greenSum * frameContext.awb.gains.green * .587 +
		       blueSum * frameContext.awb.gains.blue * .114;

	/* And return the average brightness */
	return Y_sum / (grid.height * grid.width);
}


/**
 * \brief Process IPU3 statistics, and run AGC operations
 * \param[in] context The shared IPA context
 * \param[in] stats The IPU3 statistics and ISP results
 *
 * Identify the current image brightness, and use that to estimate the optimal
 * new exposure and gain for the scene.
 */
void Agc::process(IPAContext &context, const ipu3_uapi_stats_3a *stats)
{
	/* Get the latest exposure and gain applied */
	uint32_t &exposure = context.frameContext.agc.exposure;
	double &analogueGain = context.frameContext.agc.gain;
	measureBrightness(stats, context.configuration.grid.bdsGrid);

	double currentYGain = 1.0;
	/* \todo: the target Y needs to be grabbed from a configuration */
	double targetY = 60;
	/*
	 * Do this calculation a few times as brightness increase can be
	 * non-linear when there are saturated regions.
	 */
	for (int i = 0; i < 8; i++) {
		double initialY = computeInitialY(context.frameContext,
						  context.configuration.grid.bdsGrid,
						  stats, currentYGain);
		double extra_gain = std::min(10.0, targetY / (initialY + .001));

		currentYGain *= extra_gain;
		LOG(IPU3Agc, Debug) << "Initial Y " << initialY
				    << " target " << targetY
				    << " gives gain " << currentYGain;
		if (extra_gain < 1.01)
			break;
	}

	computeExposure(exposure, analogueGain, currentYGain);
	frameCount_++;
}

} /* namespace ipa::ipu3::algorithms */

} /* namespace libcamera */
