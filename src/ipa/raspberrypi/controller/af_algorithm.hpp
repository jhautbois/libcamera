/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2022, Raspberry Pi (Trading) Limited
 *
 * af_algorithm.hpp - autofocus control algorithm interface
 */
#pragma once

#include <libcamera/geometry.h>

#include "algorithm.hpp"

namespace RPiController {

class AfAlgorithm : public Algorithm
{
public:
	AfAlgorithm(Controller *controller) : Algorithm(controller) {}
	// An af algorithm must provide the following:
	virtual void SetMode(const uint32_t &mode) = 0;
	// start a cycle (in auto mode)
	virtual void Trigger() = 0;
	// cancel a cycle (in auto mode)
	virtual void Cancel() = 0;
	// set AF windows
	virtual void SetWindows(const libcamera::Rectangle &afWindows) = 0;
	// set AF range
	virtual void SetRange(const uint32_t &low, const uint32_t &high) = 0;
	// set AF speed
	virtual void setSpeed(const uint32_t &speed) = 0;
};

} // namespace RPiController
