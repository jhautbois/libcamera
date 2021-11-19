/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Ideas On Board
 *
 * agc.cpp - AGC/AEC mean-based control algorithm
 */

#include "agc.h"

#include <algorithm>
#include <chrono>
#include <cmath>

#include <libcamera/base/log.h>

#include <libcamera/ipa/core_ipa_interface.h>

/**
 * \file agc.h
 */

namespace libcamera {

using namespace std::literals::chrono_literals;

namespace ipa::rkisp1::algorithms {

/**
 * \class Agc
 * \brief A mean-based auto-exposure algorithm
 */

LOG_DEFINE_CATEGORY(RkISP1Agc)

/* Limits for analogue gain values */
static constexpr double kMinAnalogueGain = 1.0;
static constexpr double kMaxAnalogueGain = 8.0;

/* \todo Honour the FrameDurationLimits control instead of hardcoding a limit */
static constexpr utils::Duration kMaxShutterSpeed = 60ms;

/* Number of frames to wait before calculating stats on minimum exposure */
static constexpr uint32_t kNumStartupFrames = 10;

/* Maximum luminance used for brightness normalization */
static constexpr double kMaxLuminance = 255.0;

/*
 * Normalized luma value target.
 *
 * It's a number that's chosen so that, when the camera points at a grey
 * target, the resulting image brightness is considered right.
 */
static constexpr double kNormalizedLumaTarget = 0.4;

Agc::Agc()
	: frameCount_(0), lineDuration_(0s), minShutterSpeed_(0s),
	  maxShutterSpeed_(0s), filteredExposure_(0s), currentExposure_(0s)
{
}

/**
 * \brief Configure the AGC given a configInfo
 * \param[in] context The shared IPA context
 * \param[in] configInfo The IPA configuration data
 *
 * \return 0
 */
int Agc::configure(IPAContext &context, const IPACameraSensorInfo &configInfo)
{
	/* \todo use the IPAContext to provide the limits */
	lineDuration_ = configInfo.lineLength * 1.0s / configInfo.pixelRate;

	minShutterSpeed_ = context.configuration.agc.minShutterSpeed;
	maxShutterSpeed_ = std::min(context.configuration.agc.maxShutterSpeed,
				    kMaxShutterSpeed);

	minAnalogueGain_ = std::max(context.configuration.agc.minAnalogueGain, kMinAnalogueGain);
	maxAnalogueGain_ = std::min(context.configuration.agc.maxAnalogueGain, kMaxAnalogueGain);

	/* Configure the default exposure and gain. */
	context.frameContext.agc.gain = minAnalogueGain_;
	context.frameContext.agc.exposure = 10ms / lineDuration_;

	return 0;
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

	LOG(RkISP1Agc, Debug) << "After filtering, total_exposure " << filteredExposure_;
}

/**
 * \brief Estimate the new exposure and gain values
 * \param[inout] frameContext The shared IPA frame Context
 * \param[in] currentYGain The gain calculated on the current brightness level
 */
void Agc::computeExposure(IPAFrameContext &frameContext, double currentYGain)
{
	/* Get the effective exposure and gain applied on the sensor. */
	uint32_t exposure = frameContext.sensor.exposure;
	double analogueGain = frameContext.sensor.gain;

	/* Consider within 1% of the target as correctly exposed */
	if (std::abs(currentYGain - 1.0) < 0.01)
		LOG(RkISP1Agc, Debug) << "We are well exposed (iqMean = "
				      << currentYGain << ")";

	/* extracted from Rpi::Agc::computeTargetExposure */

	/* Calculate the shutter time in seconds */
	utils::Duration currentShutter = exposure * lineDuration_;

	/*
	 * Update the exposure value for the next computation using the values
	 * of exposure and gain really used by the sensor.
	 */
	utils::Duration effectiveExposureValue = currentShutter * analogueGain;

	LOG(RkISP1Agc, Debug) << "Actual total exposure " << currentShutter * analogueGain
			      << " Shutter speed " << currentShutter
			      << " Gain " << analogueGain
			      << " Needed ev gain " << currentYGain;

	/*
	 * Calculate the current exposure value for the scene as the latest
	 * exposure value applied multiplied by the new estimated gain.
	 */
	currentExposure_ = effectiveExposureValue * currentYGain;

	/* Clamp the exposure value to the min and max authorized */
	utils::Duration maxTotalExposure = maxShutterSpeed_ * maxAnalogueGain_;
	currentExposure_ = std::min(currentExposure_, maxTotalExposure);
	LOG(RkISP1Agc, Debug) << "Target total exposure " << currentExposure_
			      << ", maximum is " << maxTotalExposure;

	/* \todo: estimate if we need to desaturate */
	filterExposure();

	/* Divide the exposure value as new exposure and gain values */
	utils::Duration exposureValue = filteredExposure_;
	utils::Duration shutterTime;

	/*
	* Push the shutter time up to the maximum first, and only then
	* increase the gain.
	*/
	shutterTime = std::clamp<utils::Duration>(exposureValue / minAnalogueGain_,
						  minShutterSpeed_, maxShutterSpeed_);
	double stepGain = std::clamp(exposureValue / shutterTime,
				     minAnalogueGain_, maxAnalogueGain_);
	LOG(RkISP1Agc, Debug) << "Divided up shutter and gain are "
			      << shutterTime << " and "
			      << stepGain;

	/* Update the estimated exposure and gain. */
	frameContext.agc.exposure = shutterTime / lineDuration_;
	frameContext.agc.gain = stepGain;
}

/**
 * \brief Estimate the average brightness of the frame
 * \param[in] ae The RkISP1 statistics and ISP results
 * \param[in] currentYGain The gain calculated on the current brightness level
 * \return The normalized luma
 */
double Agc::computeInitialY(const rkisp1_cif_isp_ae_stat *ae, double currentYGain)
{
	double ySum = 0.0;
	unsigned int num = 0;

	for (unsigned int aeCell = 0; aeCell < RKISP1_CIF_ISP_AE_MEAN_MAX_V10; aeCell++) {
		ySum += ae->exp_mean[aeCell] * currentYGain;
		num++;
	}

	/* \todo Weight with the AWB gains */

	/* Return the normalized relative luminance. */
	return ySum / num / kMaxLuminance;
}

/**
 * \brief Process RkISP1 statistics, and run AGC operations
 * \param[in] context The shared IPA context
 * \param[in] stats The RKIsp1 statistics and ISP results
 *
 * Identify the current image brightness, and use that to estimate the optimal
 * new exposure and gain for the scene.
 */
void Agc::process(IPAContext &context, const rkisp1_stat_buffer *stats)
{
	const rkisp1_cif_isp_stat *params = &stats->params;
	ASSERT(stats->meas_type & RKISP1_CIF_ISP_STAT_AUTOEXP);

	const rkisp1_cif_isp_ae_stat *ae = &params->ae;

	double currentYGain = 1.0;
	double targetY = kNormalizedLumaTarget;

	/*
	 * Do this calculation a few times as brightness increase can be
	 * non-linear when there are saturated regions.
	 */
	for (int i = 0; i < 8; i++) {
		double initialY = computeInitialY(ae, currentYGain);
		double extra_gain = std::min(10.0, targetY / (initialY + .001));

		currentYGain *= extra_gain;
		LOG(RkISP1Agc, Debug) << "Initial Y " << initialY
				      << " target " << targetY
				      << " gives gain " << currentYGain;
		if (extra_gain < 1.01)
			break;
	}

	computeExposure(context.frameContext, currentYGain);
	frameCount_++;
}

} /* namespace ipa::rkisp1::algorithms */

} /* namespace libcamera */
