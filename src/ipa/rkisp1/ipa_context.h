/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Google Inc.
 *
 * ipa_context.h - RkISP1 IPA Context
 *
 */
#ifndef __LIBCAMERA_RKISP1_IPA_CONTEXT_H__
#define __LIBCAMERA_RKISP1_IPA_CONTEXT_H__

#include <linux/rkisp1-config.h>

#include <libcamera/base/utils.h>

#include <libcamera/geometry.h>

namespace libcamera {

namespace ipa::rkisp1 {

struct IPASessionConfiguration {
	struct {
		utils::Duration minShutterSpeed;
		utils::Duration maxShutterSpeed;
		double minAnalogueGain;
		double maxAnalogueGain;
	} agc;
};

struct IPAFrameContext {
	struct {
		uint32_t exposure;
		double gain;
	} agc;

	struct {
		uint32_t exposure;
		double gain;
	} sensor;
};

struct IPAContext {
	IPASessionConfiguration configuration;
	IPAFrameContext frameContext;
};

} /* namespace ipa::rkisp1 */

} /* namespace libcamera*/

#endif /* __LIBCAMERA_RKISP1_IPA_CONTEXT_H__ */
