/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Ideas On Board
 *
 * ipu3_agc.cpp - AGC/AEC control algorithm
 */

#include "ipu3_agc.h"

#include <algorithm>
#include <numeric>

#include "libcamera/internal/log.h"

#include "libipa/histogram.h"

namespace libcamera {

namespace ipa {

LOG_DEFINE_CATEGORY(IPU3Agc)

/* Number of frames to wait before calculating stats on minimum exposure */
static const uint32_t kInitialFrameMinAECount = 0;
/* Number of frames to wait before calculating stats on maximum exposure */
static const uint32_t kInitialFrameMaxAECount = 12;
/* Number of frames to wait before calculating stats and estimate gain/exposure */
static const uint32_t kInitialFrameSkipCount = 18;
/* Number of frames to wait between new gain/exposure estimations */
static const uint32_t kFrameSkipCount = 8;

/* Maximum ISO value for analogue gain */
static const uint32_t kMaxISO = 1000;
static const uint32_t kMinISO = 100;
/* Maximum analogue gain value
 * \todo grab it from a camera helper */
static const uint32_t kMinGain = kMinISO / 100;
static const uint32_t kMaxGain = kMaxISO / 100;
/* \todo use calculated value based on sensor */
static const uint32_t kMinExposure = 1;
static const uint32_t kMaxExposure = 1976;

IPU3Agc::IPU3Agc()
	: frameCount_(0), converged_(false), updateControls_(false)
{
}

IPU3Agc::~IPU3Agc()
{
}

void IPU3Agc::initialise()
{
}

void IPU3Agc::process()
{
}
#if 0
/*
 * \todo This function is taken from numerical recipes and calculates all
 * moments. It needs to be rewritten properly and maybe in a "math" class ?
 */
void IPU3Agc::moments(std::unordered_map<uint32_t, uint32_t> &data, int n)
{
	int j;
	double ep = 0.0, s, p;
	double ave, adev, sdev;
	double var, skew, curt;

	s = 0.0;
	for (j = 1; j <= n; j++)
		s += data[j];

	ave = s / n;
	adev = var = skew = curt = 0.0;

	for (j = 1; j <= n; j++) {
		adev += s = data[j] - (ave);
		ep += s;
		var += (p = s * s);
		skew += (p *= s);
		curt += (p *= s);
	}

	adev /= n;
	var = (var - ep * ep / n) / (n - 1);
	sdev = std::sqrt(var);

	if (var) {
		skew /= n * var * sdev;
		curt = curt / (n * var * var) - 3.0;
	}
	skew_ = skew;
}
#endif
void IPU3Agc::processBrightness(Rectangle roi, const ipu3_uapi_stats_3a *stats)
{
	Point topleft = roi.topLeft();
	uint32_t startY = (topleft.y / 16) * 160 * 8;
	uint32_t startX = (topleft.x / 8) * 8;
	uint32_t endX = (startX + (roi.size().width / 8)) * 8;

	cellsBrightness_.clear();

	for (uint32_t j = (topleft.y / 16); j < (topleft.y / 16) + (roi.size().height / 16); j++) {
		for (uint32_t i = startX + startY; i < endX + startY; i += 8) {
			uint8_t Gr = stats->awb_raw_buffer.meta_data[i];
			uint8_t R = stats->awb_raw_buffer.meta_data[i + 1];
			uint8_t B = stats->awb_raw_buffer.meta_data[i + 2];
			uint8_t Gb = stats->awb_raw_buffer.meta_data[i + 3];
			cellsBrightness_.push_back(static_cast<uint32_t>(0.299 * R + 0.587 * (Gr + Gb) / 2 + 0.114 * B));
		}
	}
	std::sort(cellsBrightness_.begin(), cellsBrightness_.end());

	/* \todo create a class to generate histograms ! */
	uint32_t hist[256] = {0};
	for (uint32_t const &val : cellsBrightness_)
		hist[val]++;
	
	Histogram histogram(hist, 256);
	//moments(hist, 256);
	iqMean_ = histogram.interQuantileMean(0.25, 0.75);
	spread_ = histogram.quantile(0.75) - histogram.quantile(0.25);
	LOG(IPU3Agc, Debug) << "inter quantile mean: " << iqMean_
						<< " first: " << histogram.quantile(0.25)
						<< " last: " << histogram.quantile(0.75)
						<< " target gain: " << (0.9*256) / iqMean_
						<< " Q3-Q1: " << spread_;
}

/* \todo make this function a math one ? */
uint32_t IPU3Agc::rootApproximation(uint32_t currentValue, uint32_t prevValue, double currentMean, double prevMean)
{
	return static_cast<uint32_t>((currentValue * prevMean + prevValue * currentMean) / (prevMean + currentMean));
}

void IPU3Agc::lockExposureGain(uint32_t &exposure, uint32_t &gain)
{
	/* Algorithm initialization wait for first valid frames */
	/* \todo - have a number of frames given by DelayedControls ?
	 * - implement a function for IIR */
	if (frameCount_ == kInitialFrameMinAECount) {
		exposure = kMinExposure;
		gain = kMinGain;
		converged_ = false;
		updateControls_ = true;
	} else if (frameCount_ == kInitialFrameMaxAECount) {
		prevIqMean_ = iqMean_;
		LOG(IPU3Agc, Error) << "-> " << frameCount_
				    << "," << exposure
				    << "," << gain
				    << "," << spread_
				    << "," << iqMean_
				    << "," << prevIqMean_;
		prevExposure_ = exposure;
		prevGain_ = gain;
		gain = kMaxGain;
		exposure = kMaxExposure;
		updateControls_ = true;
	} else if (frameCount_ == kInitialFrameSkipCount) {
		updateControls_ = true;
		currentIqMean_ = iqMean_;
		LOG(IPU3Agc, Error) << "-> " << frameCount_
				    << "," << exposure
				    << "," << gain
				    << "," << spread_
				    << "," << iqMean_
				    << "," << prevIqMean_;
		nextExposure_ = std::max(kMinExposure,
					 std::min(kMaxExposure, rootApproximation(prevExposure_, currentExposure_, prevIqMean_, currentIqMean_)));
		nextGain_ = std::max(kMaxGain,
				     std::min(kMaxGain, rootApproximation(kMinGain, kMaxGain, prevIqMean_, currentIqMean_)));
	} else if ((frameCount_ > kInitialFrameSkipCount) && (frameCount_ - lastFrame_ >= kFrameSkipCount)) {
		LOG(IPU3Agc, Error) << "-> " << frameCount_
				    << "," << exposure
				    << "," << gain
				    << "," << spread_
				    << "," << iqMean_
				    << "," << prevIqMean_;
		if (std::abs(prevIqMean_ - iqMean_) < 1) {
			updateControls_ = false;
			LOG(IPU3Agc, Error) << frameCount_ << " good: " << exposure << "," << gain << "," << spread_ << "," << iqMean_ << "," << currentIqMean_ - prevIqMean_;
			converged_ = true;
		} else if (iqMean_ < 150) {
			updateControls_ = true;
			prevExposure_ = nextExposure_;
			prevGain_ = nextGain_;
			nextExposure_ = std::max(kMinExposure,
						 std::min(kMaxExposure, rootApproximation(prevExposure_, currentExposure_, prevIqMean_, currentIqMean_)));
			nextGain_ = std::max(kMaxGain,
					     std::min(kMaxGain, rootApproximation(kMinGain, kMaxGain, prevIqMean_, currentIqMean_)));
			nextExposure_ = exposure;
			nextGain_ = gain;
		} else if (iqMean_ > 210) {
			updateControls_ = true;
			currentExposure_ = nextExposure_;
			currentGain_ = nextGain_;
			nextExposure_ = std::max(kMinExposure,
						 std::min(kMaxExposure, rootApproximation(prevExposure_, currentExposure_, prevIqMean_, currentIqMean_)));
			nextGain_ = std::max(kMaxGain,
					     std::min(kMaxGain, rootApproximation(kMinGain, kMaxGain, prevIqMean_, currentIqMean_)));
			nextExposure_ = exposure;
			nextGain_ = gain;
		}
		prevIqMean_ = currentIqMean_;
		currentIqMean_ = iqMean_;
		lastFrame_ = frameCount_;
	} else {
		updateControls_ = false;
	}
}

void IPU3Agc::process(const ipu3_uapi_stats_3a *stats, uint32_t &exposure, uint32_t &gain)
{
	processBrightness(Rectangle(250, 160, 800, 400), stats);
	if (!converged_)
		lockExposureGain(exposure, gain);
	frameCount_++;
}

} /* namespace ipa */

} /* namespace libcamera */
