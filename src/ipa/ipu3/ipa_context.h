/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Ideas On Board
 *
 * ipu3_ipa_context.h - IPU3 IPA Context
 *
 * Context information shared between the algorithms
 */
#ifndef __LIBCAMERA_IPU3_IPA_CONTEXT_H__
#define __LIBCAMERA_IPU3_IPA_CONTEXT_H__

#include <linux/intel-ipu3.h>

#include <libcamera/geometry.h>

namespace libcamera {

namespace ipa::ipu3 {

struct IPAContext {
	/* Input statistics from the previous frame */
	const ipu3_uapi_stats_3a *stats;

	/* Output Parameters which will be written to the hardware */
	ipu3_uapi_params params;

	/* AWB specific parameters to share */
	struct Awb {
		struct Grid {
			/* BDS grid plane config used by the kernel */
			ipu3_uapi_grid_config bdsGrid;
			/* BDS output size configured by the pipeline handler */
			Size bdsOutputSize;
		} grid;
	} awb;
};

} /* namespace ipa::ipu3 */

} /* namespace libcamera*/

#endif /* __LIBCAMERA_IPU3_IPA_CONTEXT_H__ */
