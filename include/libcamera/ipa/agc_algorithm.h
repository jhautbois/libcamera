/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * agc_algorithm.h - AGC/AEC control algorithm interface
 */
#ifndef __LIBCAMERA_AGC_ALGORITHM_H__
#define __LIBCAMERA_AGC_ALGORITHM_H__

#include <libcamera/ipa/ipa_algorithm.h>

namespace libcamera {

class AgcAlgorithm : public IPAAlgorithm
{
public:
	AgcAlgorithm(IPAController *controller)
		: IPAAlgorithm(controller) {}
	/* An AGC algorithm must provide the following: */
	virtual unsigned int GetConvergenceFrames() const = 0;
	virtual void SetEv(double ev) = 0;
	virtual void SetFlickerPeriod(double flicker_period) = 0;
	virtual void SetFixedShutter(double fixed_shutter) = 0; // microseconds
	virtual void SetMaxShutter(double max_shutter) = 0; // microseconds
	virtual void SetFixedAnalogueGain(double fixed_analogue_gain) = 0;
	virtual void SetMeteringMode(std::string const &metering_mode_name) = 0;
	virtual void SetExposureMode(std::string const &exposure_mode_name) = 0;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_AGC_ALGORITHM_H__ */
