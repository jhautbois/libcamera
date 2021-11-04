/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Ideas On Board
 *
 * agc.h - IPU3 AGC/AEC mean-based control algorithm
 */
#ifndef __LIBCAMERA_IPU3_ALGORITHMS_AGC_H__
#define __LIBCAMERA_IPU3_ALGORITHMS_AGC_H__

#include <linux/intel-ipu3.h>

#include <libcamera/base/utils.h>

#include <libcamera/geometry.h>

#include "algorithm.h"

namespace libcamera {

struct IPACameraSensorInfo;

namespace ipa::ipu3::algorithms {

class Agc : public Algorithm
{
public:
	Agc();
	~Agc() = default;

	int configure(IPAContext &context, const IPAConfigInfo &configInfo) override;
	void process(IPAContext &context, const ipu3_uapi_stats_3a *stats) override;

private:
	void measureBrightness(const ipu3_uapi_stats_3a *stats,
			       const ipu3_uapi_grid_config &grid);
	void filterExposure();
	void computeExposure(uint32_t &exposure, double &gain, double currentYGain);
	double computeInitialY(IPAFrameContext &frameContext,
			       const ipu3_uapi_grid_config &grid,
			       const ipu3_uapi_stats_3a *stats,
			       double currentYGain);

	uint64_t frameCount_;
	uint64_t lastFrame_;

	double iqMean_;

	utils::Duration lineDuration_;
	uint32_t minExposureLines_;
	uint32_t maxExposureLines_;

	double minAnalogueGain_;
	double maxAnalogueGain_;

	utils::Duration filteredExposure_;
	utils::Duration currentExposure_;
	utils::Duration prevExposureValue_;

	uint32_t stride_;
};

} /* namespace ipa::ipu3::algorithms */

} /* namespace libcamera */

#endif /* __LIBCAMERA_IPU3_ALGORITHMS_AGC_H__ */
