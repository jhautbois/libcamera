/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * awb_algorithm.hpp - AWB control algorithm interface
 */
#ifndef __LIBCAMERA_AWB_ALGORITHM_H__
#define __LIBCAMERA_AWB_ALGORITHM_H__

#include <libcamera/ipa/ipa_algorithm.h>

namespace libcamera {

class AwbAlgorithm : public IPAAlgorithm
{
public:
	AwbAlgorithm(IPAController *controller) : IPAAlgorithm(controller) {}
	// An AWB algorithm must provide the following:
	virtual unsigned int GetConvergenceFrames() const = 0;
	virtual void SetMode(std::string const &mode_name) = 0;
	virtual void SetManualGains(double manual_r, double manual_b) = 0;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_AWB_ALGORITHM_H__ */
