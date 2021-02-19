/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * agc.cpp - AGC/AEC control algorithm
 */
#include <iostream>
#include <numeric>
#include <unordered_map>

#include "libcamera/internal/log.h"

#include "ipu3_agc.h"

using namespace libcamera;

LOG_DEFINE_CATEGORY(IPU3Agc)

#define NAME "ipu3.agc"

IPU3Agc::IPU3Agc(IPAController *controller)
	: AgcAlgorithm(controller), frameCount_(0),
	  ev_(1.0), flicker_period_(0.0),
	  max_shutter_(0), fixed_shutter_(0),
	  fixed_analogue_gain_(0.0), algoConverged_(false)
{
}

char const *IPU3Agc::Name() const
{
	return NAME;
}

unsigned int IPU3Agc::GetConvergenceFrames() const
{
	return config_.convergence_frames;
}

void IPU3Agc::SetEv(double ev)
{
	ev_ = ev;
}

void IPU3Agc::SetFlickerPeriod(double flicker_period)
{
	flicker_period_ = flicker_period;
}

void IPU3Agc::SetMaxShutter(double max_shutter)
{
	max_shutter_ = max_shutter;
}

void IPU3Agc::SetFixedShutter(double fixed_shutter)
{
	fixed_shutter_ = fixed_shutter;
}

void IPU3Agc::SetFixedAnalogueGain(double fixed_analogue_gain)
{
	fixed_analogue_gain_ = fixed_analogue_gain;
}

void IPU3Agc::SetMeteringMode(std::string const &metering_mode_name)
{
	metering_mode_name_ = metering_mode_name;
}

void IPU3Agc::SetExposureMode(std::string const &exposure_mode_name)
{
	exposure_mode_name_ = exposure_mode_name;
}

void IPU3Agc::Prepare() {}

void IPU3Agc::Process() {}

/* \todo This function is taken from numerical recipes and calculates all moments
 * It needs to be rewritten properly and maybe in a "math" class ? */
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

void IPU3Agc::processBrightness(const ipu3_uapi_stats_3a *stats)
{
	brightnessVec_.clear();

	/*\todo Replace constant values with real BDS configuration */
	for (uint32_t j = 0; j < 45; j++) {
		for (uint32_t i = 0; i < 160 * 45 * 8; i += 8) {
			uint8_t Gr = stats->awb_raw_buffer.meta_data[i];
			uint8_t R = stats->awb_raw_buffer.meta_data[i + 1];
			uint8_t B = stats->awb_raw_buffer.meta_data[i + 2];
			uint8_t Gb = stats->awb_raw_buffer.meta_data[i + 3];
			brightnessVec_.push_back(static_cast<uint32_t>(0.299 * R + 0.587 * (Gr + Gb) / 2 + 0.114 * B));
		}
	}
	std::sort(brightnessVec_.begin(), brightnessVec_.end());

	/* \todo create a class to generate histograms ! */
	std::unordered_map<uint32_t, uint32_t> hist;
	for (uint32_t const &val : brightnessVec_)
		hist[val]++;
	moments(hist, 256);
}

void IPU3Agc::lockExposure(uint32_t &exposure, uint32_t &gain)
{
	/* Algorithm initialization wait for first valid frames */
	/* \todo - have a number of frames given by DelayedControls ?
	 * - implement a function for IIR */
	if (frameCount_ == 4) {
		prevExposure_ = exposure;

		prevSkew_ = skew_;
		/* \tdo use configured values */
		exposure = 800;
		gain = 8;
		currentExposure_ = exposure;
	} else if (frameCount_ == 8) {
		currentSkew_ = skew_;
		exposure = ((currentExposure_ * prevSkew_) + (prevExposure_ * currentSkew_)) / (prevSkew_ + currentSkew_);
		nextExposure_ = exposure;
		lastFrame_ = frameCount_;
	} else if ((frameCount_ >= 12) && (frameCount_ - lastFrame_ >= 4)) {
		currentSkew_ = skew_;
		/* \todo properly calculate a gain */
		if (frameCount_ == 12)
			gain = ((8 * prevSkew_) + (1 * currentSkew_)) / (prevSkew_ + currentSkew_);

		if (currentSkew_ - prevSkew_ > 1) {
			/* under exposed */
			prevExposure_ = nextExposure_;
			exposure = ((currentExposure_ * prevSkew_) + (prevExposure_ * currentSkew_)) / (prevSkew_ + currentSkew_);
			nextExposure_ = exposure;
		} else if (currentSkew_ - prevSkew_ < -1) {
			/* over exposed */
			currentExposure_ = nextExposure_;
			exposure = ((currentExposure_ * prevSkew_) + (prevExposure_ * currentSkew_)) / (prevSkew_ + currentSkew_);
			nextExposure_ = exposure;
		} else {
			/* we have converged */
			algoConverged_ = true;
		}
		lastFrame_ = frameCount_;
		prevSkew_ = currentSkew_;
	}
}

void IPU3Agc::Process(const ipu3_uapi_stats_3a *stats, uint32_t &exposure, uint32_t &gain)
{
	processBrightness(stats);
	if (!algoConverged_)
		lockExposure(exposure, gain);
	else {
		/* Are we still well exposed ? */
		if ((skew_ < 2) || (skew_ > 4))
			algoConverged_ = false;
	}
	frameCount_++;
}

bool IPU3Agc::Converged()
{
	return algoConverged_;
}
