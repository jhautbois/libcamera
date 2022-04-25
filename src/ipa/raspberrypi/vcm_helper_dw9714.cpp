/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2022, Raspberry Pi (Trading) Limited
 * Copyright (C) 2022, Ideas On Board
 *
 * vcm_helper_dw9714.cpp - information for dw9714 vcm
 */

#include "vcm_helper.h"

using namespace RPiController;

class VcmHelperDw9714 : public VcmHelper
{
public:
	VcmHelperDw9714();
	void GetDelays(int &coarse_delay, int &fine_delay) const override;
	void GetMacroRange(int &low, int &high) const override;
};

VcmHelperDw9714::VcmHelperDw9714()
{
}

void VcmHelperDw9714::GetDelays(int &coarse_delay, int &fine_delay) const {
	coarse_delay = 2;
	fine_delay = 1;
}

void VcmHelperDw9714::GetMacroRange(int &low, int &high) const {
	low = 370;
	high = 400;
}

static VcmHelper *Create()
{
	return new VcmHelperDw9714();
}

static RegisterVcmHelper reg("dw9714", &Create);
