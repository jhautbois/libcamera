/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Based on the implementation from the Raspberry Pi IPA,
 * Copyright (C) 2019-2021, Raspberry Pi (Trading) Ltd.
 * Copyright (C) 2021, Ideas On Board
 *
 * ipu3_agc.h - IPU3 AGC/AEC control algorithm
 */
#ifndef __LIBCAMERA_IPU3_AGC_H__
#define __LIBCAMERA_IPU3_AGC_H__

#include "ipu3_common.h"

#include <array>
#include <unordered_map>

#include <linux/intel-ipu3.h>

#include <libcamera/base/utils.h>
#include <libcamera/ipa/ipu3_ipa_interface.h>
#include <libcamera/geometry.h>

#include "libipa/algorithm.h"
#include "libipa/isp.h"

namespace libcamera {

struct IPACameraSensorInfo;

namespace ipa::ipu3 {

using utils::Duration;

class IPU3Agc : public Algorithm
{
public:
	IPU3Agc();
	~IPU3Agc() = default;

	void initialise(struct ipu3_uapi_grid_config &bdsGrid, const IPAConfigInfo &configInfo);
	void process(const ipu3_uapi_stats_3a *stats, uint32_t &exposure, double &analogueGain);
	bool converged() { return converged_; }
	bool updateControls() { return updateControls_; }
	/* \todo Use a metadata exchange between IPAs */
	double gamma() { return gamma_; }

private:
	void processBrightness(const ipu3_uapi_stats_3a *stats);
	void filterExposure();
	void lockExposureGain(uint32_t &exposure, double &gain);
	void generateStats(const ipu3_uapi_stats_3a *stats);
	void clearStats();
	void generateZones(std::vector<RGB> &zones);
	double computeInitialY(IspStatsRegion regions[], AwbStatus const &awb, double weights[], double gain);
	void computeTargetExposure(double currentGain);
	void divideUpExposure();
	void computeGain(double &currentGain);

	AwbStatus awb_;
	double weights_[kNumAgcWeightedZones];
	IspStatsRegion agcStats_[kNumAgcWeightedZones];

	struct ipu3_uapi_grid_config aeGrid_;
	ControlInfoMap ctrls_;

	uint32_t minExposure_;
	uint32_t maxExposure_;

	uint32_t minGain_;
	uint32_t maxGain_;

	uint64_t frameCount_;
	uint64_t lastFrame_;

	bool converged_;
	bool updateControls_;

	double iqMean_;
	double gamma_;

	Duration lineDuration_;
	Duration maxExposureTime_;

	Duration prevExposure_;
	Duration prevExposureNoDg_;
	Duration currentExposure_;
	Duration currentExposureNoDg_;

	Duration currentShutter_;
	std::vector<Duration> shutterConstraints_;
	Duration fixedShutter_;
	Duration filteredShutter_;

	double currentAnalogueGain_;
	std::vector<double> gainConstraints_;
	double fixedAnalogueGain_;
	double filteredAnalogueGain_;
};

} /* namespace ipa::ipu3 */

} /* namespace libcamera */

#endif /* __LIBCAMERA_IPU3_AGC_H__ */
