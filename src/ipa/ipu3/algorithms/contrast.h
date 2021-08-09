/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Google
 *
 * constrast.h - IPU3 Contrast and Gamma control
 */
#ifndef __LIBCAMERA_IPU3_ALGORITHMS_CONTRAST_H__
#define __LIBCAMERA_IPU3_ALGORITHMS_CONTRAST_H__

#include "algorithm.h"

namespace libcamera {

namespace ipa::ipu3::algorithms {

class Contrast : public Algorithm
{
public:
	Contrast();
	~Contrast() = default;

	int initialise(IPAContext &context) override;

private:
	double gamma_;
};

} /* namespace ipa::ipu3::algorithms */

} /* namespace libcamera */

#endif /* __LIBCAMERA_IPU3_ALGORITHMS_CONTRAST_H__ */
