/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Ideas On Board
 *
 * ipu3_agc.h - IPU3 AGC/AEC control algorithm
 */
#ifndef __LIBCAMERA_IPU3_ALGORITHMS_AGC_H__
#define __LIBCAMERA_IPU3_ALGORITHMS_AGC_H__

#include <linux/intel-ipu3.h>

#include <array>
#include <unordered_map>

#include <libcamera/base/utils.h>

#include <libcamera/geometry.h>

#include "algorithm.h"

namespace libcamera {

struct IPACameraSensorInfo;

namespace ipa::ipu3::algorithms {

using utils::Duration;

class Agc : public Algorithm
{
public:
	Agc();
	~Agc() = default;

	int configure(IPAContext &context) override;
	void process(IPAContext &context) override;

	bool converged() { return converged_; }
	bool updateControls() { return updateControls_; }

private:
	void processBrightness(const ipu3_uapi_stats_3a *stats);
	void filterExposure();
	void lockExposureGain(IPAContext &context);

	struct ipu3_uapi_grid_config aeGrid_;

	uint64_t frameCount_;
	uint64_t lastFrame_;

	bool converged_;
	bool updateControls_;

	double iqMean_;

	Duration maxExposureTime_;

	Duration prevExposure_;
	Duration prevExposureNoDg_;
	Duration currentExposure_;
	Duration currentExposureNoDg_;
};

} /* namespace ipa::ipu3::algorithms */

} /* namespace libcamera */

#endif /* __LIBCAMERA_IPU3_ALGORITHMS_AGC_H__ */
