/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Ideas On Board
 *
 * algorithm.h - IPU3 control algorithm interface
 */
#ifndef __LIBCAMERA_IPA_IPU3_ALGORITHM_H__
#define __LIBCAMERA_IPA_IPU3_ALGORITHM_H__

#include <ipa_context.h>

namespace libcamera {

namespace ipa::ipu3 {

class Algorithm
{
public:
	virtual ~Algorithm() {}

	virtual int initialise([[maybe_unused]] IPAContext &context) { return 0; }
	virtual int configure([[maybe_unused]] IPAContext &context) { return 0; }
	virtual void process([[maybe_unused]] IPAContext &context) {}
};

} /* namespace ipa::ipu3 */

} /* namespace libcamera */

#endif /* __LIBCAMERA_IPA_IPU3_ALGORITHM_H__ */
