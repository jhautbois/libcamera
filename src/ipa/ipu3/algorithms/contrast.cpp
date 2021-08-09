/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Ideas On Board
 *
 * constrast.cpp - IPU3 Contrast and Gamma control
 */

#include "contrast.h"

#include <cmath>

#include <libcamera/base/log.h>

namespace libcamera {

namespace ipa::ipu3::algorithms {

LOG_DEFINE_CATEGORY(IPU3Contrast)

Contrast::Contrast()
	: gamma_(1.0)
{
	LOG(IPU3Contrast, Info) << "Instantiate Gamma";
}

int Contrast::initialise(IPAContext &context)
{
	ipu3_uapi_params &params = context.params;

	params.use.acc_gamma = 1;
	params.acc_param.gamma.gc_ctrl.enable = 1;

	/* Limit the gamma effect for now */
	gamma_ = 1.1;

	/* Plot the gamma curve into the look up table */
	for (uint32_t i = 0; i < 256; i++) {
		double j = i / 255.0;
		double gamma = std::pow(j, 1.0 / gamma_);

		/* The maximum value 255 is represented on 13 bits in the IPU3 */
		params.acc_param.gamma.gc_lut.lut[i] = gamma * 8191;
	}

	LOG(IPU3Contrast, Info) << "Processed Gamma Curve";

	return 0;
}

} /* namespace ipa::ipu3::algorithms */

} /* namespace libcamera */
