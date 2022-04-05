/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2022, Raspberry Pi (Trading) Limited
 * Copyright (C) 2022, Ideas On Board
 *
 * vcm_helper.cpp - helper information for different VCMs
 */

#include <map>

#include <libcamera/base/log.h>

#include "vcm_helper.h"

using namespace RPiController;
using namespace libcamera;
using libcamera::utils::Duration;

namespace libcamera {
LOG_DECLARE_CATEGORY(IPARPI)
}

static std::map<std::string, VcmHelperCreateFunc> vcm_helpers;

VcmHelper *VcmHelper::Create(std::string const &vcm_name)
{
	/*
	 * VcmHelpers get registered by static RegisterVcmHelper
	 * initialisers.
	 */
	for (auto &p : vcm_helpers) {
		if (vcm_name.find(p.first) != std::string::npos)
			return p.second();
	}

	return nullptr;
}

VcmHelper::VcmHelper()
{
}

VcmHelper::~VcmHelper()
{
}

void VcmHelper::GetDelays(int &coarse_delay, int &fine_delay) const {
	coarse_delay = 1;
	fine_delay = 1;
}

void VcmHelper::GetMacroRange(int &low, int &high) const {
	low = 200;
	high = 600;
}

RegisterVcmHelper::RegisterVcmHelper(char const *vcm_name,
				     VcmHelperCreateFunc create_func)
{
	vcm_helpers[std::string(vcm_name)] = create_func;
}
