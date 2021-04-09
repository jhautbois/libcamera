/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Ideas On Board
 *
 * algorithm.h - ISP control algorithm interface
 */
#ifndef __LIBCAMERA_IPA_LIBIPA_ALGORITHM_H__
#define __LIBCAMERA_IPA_LIBIPA_ALGORITHM_H__

namespace libcamera {

namespace ipa {

class Algorithm
{
public:
	virtual ~Algorithm();

	struct ispStatsRegion {
		unsigned int counted;
		unsigned int notcounted;
		unsigned long long rSum;
		unsigned long long gSum;
		unsigned long long bSum;
	};
};

} /* namespace ipa */

} /* namespace libcamera */

#endif /* __LIBCAMERA_IPA_LIBIPA_ALGORITHM_H__ */
