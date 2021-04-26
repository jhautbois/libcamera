/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Ideas On Board
 *
 * awb.h - Common AWB functions
 */
#ifndef __LIBCAMERA_IPA_AWB_H__
#define __LIBCAMERA_IPA_AWB_H__

#include <stdint.h>

#include <vector>

#include "isp.h"

namespace libcamera {

namespace ipa {

uint32_t estimateCCT(double red, double green, double blue);
void awbGreyWorld(std::vector<RGB> &zones, AwbStatus &results);

} /* namespace ipa */

} /* namespace libcamera*/
#endif /* __LIBCAMERA_IPA_AWB_H__ */
