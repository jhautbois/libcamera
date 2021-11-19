/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Ideas On Board
 *
 * agc.h - RkISP1 AGC/AEC mean-based control algorithm
 */
#ifndef __LIBCAMERA_RKISP1_ALGORITHMS_AGC_H__
#define __LIBCAMERA_RKISP1_ALGORITHMS_AGC_H__

#include <linux/rkisp1-config.h>

#include <libcamera/base/utils.h>

#include <libcamera/geometry.h>

#include "algorithm.h"

namespace libcamera {

struct IPACameraSensorInfo;

namespace ipa::rkisp1::algorithms {

class Agc : public Algorithm
{
public:
	Agc();
	~Agc() = default;

	int configure(IPAContext &context, const IPACameraSensorInfo &configInfo) override;
	void prepare(IPAContext &context, rkisp1_params_cfg *params) override;
	void process(IPAContext &context, const rkisp1_stat_buffer *stats) override;

private:
	void filterExposure();
	void computeExposure(IPAFrameContext &frameContext, double currentYGain);
	double computeInitialY(const rkisp1_cif_isp_ae_stat *ae, double currentYGain);

	uint64_t frameCount_;

	utils::Duration lineDuration_;
	utils::Duration minShutterSpeed_;
	utils::Duration maxShutterSpeed_;

	double minAnalogueGain_;
	double maxAnalogueGain_;

	uint32_t numCells_;

	utils::Duration filteredExposure_;
	utils::Duration currentExposure_;
};

} /* namespace ipa::rkisp1::algorithms */

} /* namespace libcamera */

#endif /* __LIBCAMERA_RKISP1_ALGORITHMS_AGC_H__ */
