/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Ideas On Board
 *
 * algorithm.h - RkISP1 control algorithm interface
 */
#ifndef __LIBCAMERA_IPA_RKISP1_ALGORITHM_H__
#define __LIBCAMERA_IPA_RKISP1_ALGORITHM_H__

#include <libcamera/ipa/rkisp1_ipa_interface.h>

#include "ipa_context.h"

namespace libcamera {

namespace ipa::rkisp1 {

class Algorithm
{
public:
	virtual ~Algorithm() {}

	virtual int configure(IPAContext &context, const IPACameraSensorInfo &configInfo);
	virtual void prepare(IPAContext &context, rkisp1_params_cfg *params);
	virtual void process(IPAContext &context, const rkisp1_stat_buffer *stats);
};

} /* namespace ipa::rkisp1 */

} /* namespace libcamera */

#endif /* __LIBCAMERA_IPA_RKISP1_ALGORITHM_H__ */
