/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * ipu3_agc.h - IPU3 AGC/AEC control algorithm
 */
#ifndef __LIBCAMERA_IPU3_AGC_H__
#define __LIBCAMERA_IPU3_AGC_H__

#include <unordered_map>
#include <vector>

#include <linux/intel-ipu3.h>

#include <libcamera/geometry.h>

#include "libipa/algorithm.h"

namespace libcamera {

namespace ipa {

class IPU3Agc : public Algorithm
{
public:
	IPU3Agc();
	~IPU3Agc();

	void initialise() override;
	void process() override;

	void process(const ipu3_uapi_stats_3a *stats, uint32_t &exposure, uint32_t &gain);
	bool converged() { return converged_; }

private:
	void moments(std::unordered_map<uint32_t, uint32_t> &data, int n);
	void processBrightness(const ipu3_uapi_stats_3a *stats);
	uint32_t rootApproximation();
	void lockExposure(uint32_t &exposure, uint32_t &gain);

	uint64_t frameCount_;
	uint64_t lastFrame_;

	/* Vector of calculated brightness for each cell */
	std::vector<uint32_t> cellsBrightness_;

	/* Values for filtering */
	uint32_t prevExposure_;
	uint32_t currentExposure_;
	uint32_t nextExposure_;

	double skew_;
	double prevSkew_;
	double currentSkew_;
	bool converged_;
};

} /* namespace ipa */

} /* namespace libcamera */

#endif /* __LIBCAMERA_IPU3_AGC_H__ */
