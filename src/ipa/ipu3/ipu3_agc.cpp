/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * ipu3_agc.cpp - AGC/AEC control algorithm
 */

#include "ipu3_agc.h"

#include <numeric>

#include "libcamera/internal/log.h"

namespace libcamera {

namespace ipa {

LOG_DEFINE_CATEGORY(IPU3Agc)

/* Number of frames to wait before calculating stats on minimum exposure */
static const uint32_t kInitialFrameMinAECount = 4;
/* Number of frames to wait before calculating stats on maximum exposure */
static const uint32_t kInitialFrameMaxAECount = 8;
/* Number of frames to wait before calculating stats and estimate gain/exposure */
static const uint32_t kInitialFrameSkipCount = 12;

/* Number of frames to wait between new gain/exposure estimations */
static const uint32_t kFrameSkipCount = 4;

IPU3Agc::IPU3Agc()
	: frameCount_(0), converged_(false)
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
		s += data[j] * j;

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

void IPU3Agc::processBrightness(const ipu3_uapi_stats_3a *stats)
{
	cellsBrightness_.clear();

	/*\todo Replace constant values with real BDS configuration */
	for (uint32_t i = 0; i < 160 * 45 * 8; i += 8) {
		uint8_t Gr = stats->awb_raw_buffer.meta_data[i];
		uint8_t R = stats->awb_raw_buffer.meta_data[i + 1];
		uint8_t B = stats->awb_raw_buffer.meta_data[i + 2];
		uint8_t Gb = stats->awb_raw_buffer.meta_data[i + 3];
		cellsBrightness_.push_back(static_cast<uint32_t>(0.299 * R + 0.587 * (Gr + Gb) / 2 + 0.114 * B));
	}
	std::sort(cellsBrightness_.begin(), cellsBrightness_.end());

	/* \todo create a class to generate histograms ! */
	std::unordered_map<uint32_t, uint32_t> hist;
	for (uint32_t const &val : cellsBrightness_)
		hist[val]++;
	moments(hist, 256);
}

/* \todo make this function a math one ? */
uint32_t IPU3Agc::rootApproximation()
{
	return (currentExposure_ * prevSkew_ + prevExposure_ * currentSkew_) / (prevSkew_ + currentSkew_);
}

void IPU3Agc::lockExposure(uint32_t &exposure, uint32_t &gain)
{
	/* Algorithm initialization wait for first valid frames */
	/* \todo - have a number of frames given by DelayedControls ?
	 * - implement a function for IIR */
	if (frameCount_ == kInitialFrameMinAECount) {
		prevExposure_ = exposure;

		prevSkew_ = skew_;
		/* \todo use configured values */
		exposure = 230;
		gain = 160;
		currentExposure_ = exposure;
	} else if (frameCount_ == kInitialFrameMaxAECount) {
		currentSkew_ = skew_;
		exposure = rootApproximation();
		nextExposure_ = exposure;
		lastFrame_ = frameCount_;
	} else if ((frameCount_ >= kInitialFrameSkipCount) && (frameCount_ - lastFrame_ >= kFrameSkipCount)) {
		currentSkew_ = skew_;
		/* \todo properly calculate a gain */
		if (frameCount_ == kInitialFrameSkipCount)
			gain = ((8 * prevSkew_) + (1 * currentSkew_)) / (prevSkew_ + currentSkew_);

		if (currentSkew_ - prevSkew_ > 1) {
			/* under exposed */
			prevExposure_ = nextExposure_;
			exposure = rootApproximation();
			nextExposure_ = exposure;
		} else if (currentSkew_ - prevSkew_ < -1) {
			/* over exposed */
			currentExposure_ = nextExposure_;
			exposure = rootApproximation();
			nextExposure_ = exposure;
		} else {
			/* we have converged */
			converged_ = true;
		}
		lastFrame_ = frameCount_;
		prevSkew_ = currentSkew_;
	}
}

void IPU3Agc::process(const ipu3_uapi_stats_3a *stats, uint32_t &exposure, uint32_t &gain)
{
	processBrightness(stats);
	if (!converged_)
		lockExposure(exposure, gain);
	else {
		/* Are we still well exposed ? */
		if ((skew_ < 2) || (skew_ > 4))
			converged_ = false;
	}
	frameCount_++;
}

} /* namespace ipa */

} /* namespace libcamera */
