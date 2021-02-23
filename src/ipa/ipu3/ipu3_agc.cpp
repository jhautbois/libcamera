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
static const uint32_t kInitialFrameMinAECount = 4;
/* Number of frames to wait before calculating stats on maximum exposure */
static const uint32_t kInitialFrameMaxAECount = 8;
/* Number of frames to wait before calculating stats and estimate gain/exposure */
static const uint32_t kInitialFrameSkipCount = 12;
/* Number of frames to wait between new gain/exposure estimations */
static const uint32_t kFrameSkipCount = 4;

/* Maximum ISO value for analogue gain */
static const uint32_t kMaxISO = 1000;
static const uint32_t kMinISO = 100;
/* Maximum analogue gain value
 * \todo grab it from a camera helper */
static const uint32_t kMinGain = 16 * (kMinISO / 100);
static const uint32_t kMaxGain = 16 * (kMaxISO / 100);
/* \todo use calculated value based on sensor */
static const uint32_t kMinExposure = 1;
static const uint32_t kMaxExposure = 230;

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
void IPU3Agc::processBrightness(const ipu3_uapi_stats_3a *stats)
{
	cellsBrightness_.clear();
	
	/*\todo Replace constant values with real BDS configuration */
	for (uint32_t j = 0; j < 45; j++) {
		for (uint32_t i = 0; i < 160 * 45 * 8; i += 8) {
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
uint32_t IPU3Agc::rootApproximation()
{
	return (currentExposure_ * prevIqMean_ + prevExposure_ * currentIqMean_) / (prevIqMean_ + currentIqMean_);
}

static uint32_t getGain(uint32_t x0, double y0,
						uint32_t x1, double y1, uint32_t y)
{
	double a = (y1 - y0) / (x1 - x0);
	double b = y1 - a * x1;

	LOG(IPU3Agc, Error) << "new gain: " << x0
						<< "," << y0
						<< "," << x1
						<< "," << y1
						<< "," << a
						<< "," << b
						<< "," << (y-b) / a;

	if (a != 0)
		return (y - b) / a;
	else
		return kMinGain;
}

void IPU3Agc::lockExposureGain(uint32_t &exposure, uint32_t &gain)
{
	/* Algorithm initialization wait for first valid frames */
	/* \todo - have a number of frames given by DelayedControls ?
	 * - implement a function for IIR */
	LOG(IPU3Agc, Error) << "-> " << frameCount_
			<< "," << exposure
			<< "," << gain
			<< "," << spread_
			<< "," << prevIqMean_
			<< "," << currentIqMean_;
	if (frameCount_ == kInitialFrameMinAECount) {
		exposure = kMaxExposure;
		gain = kMinGain;
		converged_ = false;
	} else if (frameCount_ == kInitialFrameMaxAECount) {
		prevIqMean_ = spread_;
		lastFrame_ = frameCount_;
		gain = kMaxGain;
		prevExposure_ = exposure;
	} else if ((frameCount_ >= kInitialFrameSkipCount) && (frameCount_ - lastFrame_ >= kFrameSkipCount)) {
		currentIqMean_ = spread_;

		if (frameCount_ == kInitialFrameSkipCount) {
			gain = getGain(kMinGain, prevIqMean_, kMaxGain, currentIqMean_, 200);
			exposure = kMinExposure;
		}
		LOG(IPU3Agc, Error) << frameCount_
			<< "," << exposure
			<< "," << gain
			<< "," << spread_
			<< "," << currentIqMean_ - prevIqMean_;

#if 0
		if (currentIqMean_ - prevIqMean_ > 1) {
			prevExposure_ = nextExposure_;
			exposure = rootApproximation();
			LOG(IPU3Agc, Error) << frameCount_ << " under: " << prevExposure_
								<< ">" << exposure
								<< "," << gain
								<< "," << spread_
								<< "," << iqMean_
								<< "," << currentIqMean_ - prevIqMean_;
			nextExposure_ = exposure;
		}
		else if (currentIqMean_ - prevIqMean_ < -1) {
			currentExposure_ = nextExposure_;
			exposure = rootApproximation();
			LOG(IPU3Agc, Error) << frameCount_ << " over: " << prevExposure_
								<< ">" << exposure
								<< "," << gain
								<< "," << spread_
								<< "," << iqMean_
								<< "," << currentIqMean_ - prevIqMean_;
			nextExposure_ = exposure;
		} else {
			LOG(IPU3Agc, Error) << frameCount_ << " good: " << exposure << "," << gain << "," << spread_ << "," << iqMean_ << "," << currentIqMean_ - prevIqMean_;
			//converged_ = true;
		}
#endif
		//prevIqMean_ = currentIqMean_;
		lastFrame_ = frameCount_;
	}
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
		exposure = 800;
		gain = 8;
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
		lockExposureGain(exposure, gain);
	frameCount_++;
}

} /* namespace ipa */

} /* namespace libcamera */
