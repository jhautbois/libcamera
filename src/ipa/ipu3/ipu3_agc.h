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

#include "ipu3_awb.h"

#include <array>
#include <unordered_map>

#include <linux/intel-ipu3.h>

#include <libcamera/base/utils.h>
#include <libcamera/ipa/ipu3_ipa_interface.h>
#include <libcamera/geometry.h>

#include "libipa/algorithm.h"
#include "libipa/metadata.h"

namespace libcamera {

struct IPACameraSensorInfo;

namespace ipa::ipu3 {

using utils::Duration;

struct AgcResults {
	Duration shutterTime;
	double analogueGain;
};

/* List of all Metadata tags for AGC*/
static constexpr Tag<AgcResults> tagAgcResults{ "agc.results" };
static constexpr Tag<double> tagGamma{ "agc.gamma" };

/* Number of weighted zones for metering */
static constexpr uint32_t kNumAgcWeightedZones = 15;

class IPU3Agc : public Algorithm
{
public:
	IPU3Agc();
	~IPU3Agc() = default;

	void initialise(struct ipu3_uapi_grid_config &bdsGrid, const IPAConfigInfo &configInfo);
	void process(const ipu3_uapi_stats_3a *stats, Metadata *imageMetadata);

private:
	void processBrightness(const ipu3_uapi_stats_3a *stats);
	void filterExposure();
	void lockExposureGain(uint32_t &exposure, double &gain);
	void generateStats(const ipu3_uapi_stats_3a *stats);
	void clearStats();
	void generateZones(std::vector<RGB> &zones);
	double computeInitialY(StatsRegion regions[], AwbResults const &awb, double weights[], double gain);
	void computeTargetExposure(double currentGain);
	void divideUpExposure();
	void computeGain(double &currentGain);

	AwbResults awb_;
	double weights_[kNumAgcWeightedZones];
	StatsRegion agcStats_[kNumAgcWeightedZones];

	struct ipu3_uapi_grid_config aeGrid_;
	ControlInfoMap ctrls_;

	double iqMean_;
	double gamma_;

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

	AgcResults status_;
};

} /* namespace ipa::ipu3 */

} /* namespace libcamera */

#endif /* __LIBCAMERA_IPU3_AGC_H__ */
