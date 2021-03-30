/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Ideas On Board
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

	void initialise(struct ipu3_uapi_grid_config &bdsGrid);
	void process(const ipu3_uapi_stats_3a *stats, uint32_t &exposure, uint32_t &gain);
	bool converged() { return converged_; }
	bool updateControls() { return updateControls_; }
	/* \todo Use a metadata exchange between IPAs */
	double gamma() { return gamma_; }

private:
	void processBrightness(const ipu3_uapi_stats_3a *stats);
	void filterExposure(bool desaturate);
	void lockExposureGain(uint32_t &exposure, uint32_t &gain);

	struct ipu3_uapi_grid_config aeGrid_;

	uint64_t frameCount_;
	uint64_t lastFrame_;

	/* Vector of calculated brightness for each cell */
	std::vector<uint32_t> cellsBrightness_;

	bool converged_;
	bool updateControls_;

	double iqMean_;
	double gamma_;
	uint32_t histLow_;
	uint32_t histHigh_;

	double prevTotalExposure_;
	double prevTotalExposureNoDg_;
	double currentTotalExposure_;
	double currentTotalExposureNoDg_;
};

} /* namespace ipa */

} /* namespace libcamera */

#endif /* __LIBCAMERA_IPU3_AGC_H__ */
