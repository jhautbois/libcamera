/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Ideas On Board
 *
 * isp.h - ISP statistics interface
 */
#ifndef __LIBCAMERA_IPA_IPU3_COMMON_H__
#define __LIBCAMERA_IPA_IPU3_COMMON_H__

#include <cstdint>

namespace libcamera {

namespace ipa {

/* Region size for the statistics generation algorithm */
static constexpr uint32_t kAwbStatsSizeX = 16;
static constexpr uint32_t kAwbStatsSizeY = 12;

static constexpr uint32_t kMinGreenLevelInZone = 16;

struct Ipu3AwbCell {
	unsigned char greenRedAvg;
	unsigned char redAvg;
	unsigned char blueAvg;
	unsigned char greenBlueAvg;
	unsigned char satRatio;
	unsigned char padding[3];
} __attribute__((packed));

} /* namespace ipa */

} /* namespace libcamera */

#endif /* __LIBCAMERA_IPA_IPU3_COMMON_H__ */