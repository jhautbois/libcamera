/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Ideas On Board
 *
 * ipu3_awb.h - IPU3 AWB control algorithm
 */
#ifndef __LIBCAMERA_IPU3_AWB_H__
#define __LIBCAMERA_IPU3_AWB_H__

#include <vector>

#include <linux/intel-ipu3.h>

#include <libcamera/geometry.h>

#include "libipa/algorithm.h"

namespace libcamera {

namespace ipa {

static constexpr uint32_t kAwbStatsSizeX = 16;
static constexpr uint32_t kAwbStatsSizeY = 12;

class IPU3Awb : public Algorithm
{
public:
	IPU3Awb();
	~IPU3Awb();

	void initialise(ipu3_uapi_params &params, const Size &bdsOutputSize, struct ipu3_uapi_grid_config &bdsGrid);
	void calculateWBGains(const ipu3_uapi_stats_3a *stats);
	void updateWbParameters(ipu3_uapi_params &params, double agcGamma);
	struct RGB {
		RGB(double _R = 0, double _G = 0, double _B = 0)
			: R(_R), G(_G), B(_B)
		{
		}
		double R, G, B;
		RGB &operator+=(RGB const &other)
		{
			R += other.R, G += other.G, B += other.B;
			return *this;
		}
	};

	struct AwbStatus {
		double temperature_K;
		double redGain;
		double greenGain;
		double blueGain;
	};

private:
	void generateZones(std::vector<RGB> &zones);
	void generateAwbStats(const ipu3_uapi_stats_3a *stats);
	void clearAwbStats();
	void awbGrey();
	uint32_t estimateCCT(uint8_t red, uint8_t green, uint8_t blue);

	/* WB calculated gains */
	uint16_t wbGains_[4];
	uint32_t cct_;
	struct ipu3_uapi_grid_config awbGrid_;
	uint32_t frame_count_;
	std::vector<RGB> zones_;
	ispStatsRegion awbStats_[kAwbStatsSizeX * kAwbStatsSizeY];
	AwbStatus asyncResults_;
};

} /* namespace ipa */

} /* namespace libcamera*/
#endif /* __LIBCAMERA_IPU3_AWB_H__ */
