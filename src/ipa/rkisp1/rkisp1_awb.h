/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Ideas On Board
 *
 * rkisp1_awb.h - RKISP1 AWB control algorithm
 */
#ifndef __LIBCAMERA_RKISP1_AWB_H__
#define __LIBCAMERA_RKISP1_AWB_H__

#include <vector>

#include <linux/rkisp1-config.h>

#include <libcamera/geometry.h>

#include "libipa/algorithm.h"
#include "libipa/isp.h"

namespace libcamera {

namespace ipa::rkisp1 {

/* Region size for the statistics generation algorithm */
static constexpr uint32_t kAwbStatsSizeX = 16;
static constexpr uint32_t kAwbStatsSizeY = 12;

class RkISP1Awb : public Algorithm
{
public:
	RkISP1Awb();
	~RkISP1Awb() = default;

	void initialise([[maybe_unused]] rkisp1_params_cfg &params);
	void calculateWBGains(const rkisp1_stat_buffer *stats);
	void updateWbParameters(rkisp1_params_cfg &params);

	struct RkISP1AwbCell {
		unsigned char greenRedAvg;
		unsigned char redAvg;
		unsigned char blueAvg;
		unsigned char greenBlueAvg;
		unsigned char satRatio;
		unsigned char padding[3];
	} __attribute__((packed));

private:
	void generateZones(std::vector<RGB> &zones);
	void generateAwbStats(const rkisp1_stat_buffer *stats);
	void clearAwbStats();

	std::vector<RGB> zones_;
	IspStatsRegion awbStats_[kAwbStatsSizeX * kAwbStatsSizeY];
	AwbStatus asyncResults_;
};

} /* namespace ipa::rkisp1 */

} /* namespace libcamera*/
#endif /* __LIBCAMERA_RKISP1_AWB_H__ */
