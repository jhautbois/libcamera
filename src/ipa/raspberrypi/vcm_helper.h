/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2022, Raspberry Pi (Trading) Limited
 * Copyright (C) 2022, Ideas On Board
 *
 * vcm_helper.hpp - helper class providing VCM information
 */
#pragma once

#include <string>

namespace RPiController {

// The VcmHelper class provides a number of facilities that anyone trying
// to drive a VCM will need to know, but which are not provided by the
// standard driver framework. Specifically, it provides:
//
// The ability to convert the lens range mode into VCM focus values.
//
// A function to return the number of frames of delay between updating the lens
// position and for the changes to take effect.

class VcmHelper
{
public:
	static VcmHelper *Create(std::string const &vcm_name);
	VcmHelper();
	virtual ~VcmHelper();
	virtual void GetDelays(int &coarse_delay, int &fine_delay) const;
	virtual void GetMacroRange(int &low, int &high) const;
};

// This is for registering vcm helpers with the system, so that the
// VcmHelper::Create function picks them up automatically.

typedef VcmHelper *(*VcmHelperCreateFunc)();
struct RegisterVcmHelper
{
	RegisterVcmHelper(char const *vcm_name,
			  VcmHelperCreateFunc create_func);
};

} // namespace RPi
