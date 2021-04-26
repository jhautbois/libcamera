/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Ideas On Board
 *
 * ipu3_awb.cpp - AWB control algorithm
 */
#include "rkisp1_awb.h"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <unordered_map>

#include "libcamera/internal/log.h"

#include "libipa/awb.h"

namespace libcamera {

namespace ipa::rkisp1 {

LOG_DEFINE_CATEGORY(RkISP1Awb)

static constexpr uint32_t kMinZonesCounted = 16;
static constexpr uint32_t kMinGreenLevelInZone = 16;

/**
 * \struct IspStatsRegion
 * \brief RGB statistics for a given region
 *
 * The IspStatsRegion structure is intended to abstract the ISP specific
 * statistics and use an agnostic algorithm to compute AWB.
 *
 * \var IspStatsRegion::counted
 * \brief Number of pixels used to calculate the sums
 *
 * \var IspStatsRegion::uncounted
 * \brief Remaining number of pixels in the region
 *
 * \var IspStatsRegion::rSum
 * \brief Sum of the red values in the region
 *
 * \var IspStatsRegion::gSum
 * \brief Sum of the green values in the region
 *
 * \var IspStatsRegion::bSum
 * \brief Sum of the blue values in the region
 */

/**
 * \struct AwbStatus
 * \brief AWB parameters calculated
 *
 * The AwbStatus structure is intended to store the AWB
 * parameters calculated by the algorithm
 *
 * \var AwbStatus::temperatureK
 * \brief Color temperature calculated
 *
 * \var AwbStatus::redGain
 * \brief Gain calculated for the red channel
 *
 * \var AwbStatus::greenGain
 * \brief Gain calculated for the green channel
 *
 * \var AwbStatus::blueGain
 * \brief Gain calculated for the blue channel
 */

/**
 * \struct RkISP1AwbCell
 * \brief Memory layout for each cell in AWB metadata
 *
 * The RkISP1AwbCell structure is used to get individual values
 * such as red average or saturation ratio in a particular cell.
 *
 * \var RkISP1AwbCell::greenRedAvg
 * \brief Green average for red lines in the cell
 *
 * \var RkISP1AwbCell::redAvg
 * \brief Red average in the cell
 *
 * \var RkISP1AwbCell::blueAvg
 * \brief blue average in the cell
 *
 * \var RkISP1AwbCell::greenBlueAvg
 * \brief Green average for blue lines
 *
 * \var RkISP1AwbCell::satRatio
 * \brief Saturation ratio in the cell
 *
 * \var RkISP1AwbCell::padding
 * \brief array of unused bytes for padding
 */

RkISP1Awb::RkISP1Awb()
	: Algorithm()
{
	asyncResults_.blueGain = 1.0;
	asyncResults_.greenGain = 1.0;
	asyncResults_.redGain = 1.0;
	asyncResults_.temperatureK = 4500;
}

void RkISP1Awb::initialise([[maybe_unused]] rkisp1_params_cfg &params)
{
	zones_.reserve(kAwbStatsSizeX * kAwbStatsSizeY);
}

/* Generate an RGB vector with the average values for each region */
void RkISP1Awb::generateZones(std::vector<RGB> &zones)
{
	for (unsigned int i = 0; i < kAwbStatsSizeX * kAwbStatsSizeY; i++) {
		RGB zone;
		double counted = awbStats_[i].counted;
		if (counted >= kMinZonesCounted) {
			zone.G = awbStats_[i].gSum / counted;
			if (zone.G >= kMinGreenLevelInZone) {
				zone.R = awbStats_[i].rSum / counted;
				zone.B = awbStats_[i].bSum / counted;
				zones.push_back(zone);
			}
		}
	}
}

/* Translate the RkISP1 statistics into the default statistics region array */
void RkISP1Awb::generateAwbStats(const rkisp1_stat_buffer *stats)
{
	const rkisp1_cif_isp_stat *params = &stats->params;
	LOG(RkISP1Awb, Error) << "Measured AWB : "
			      << " count: " << (double)params->awb.awb_mean[0].cnt
			      << " mean G " << (double)params->awb.awb_mean[0].mean_y_or_g
			      << " mean B " << (double)params->awb.awb_mean[0].mean_cb_or_b
			      << " mean R " << (double)params->awb.awb_mean[0].mean_cr_or_r;

	awbStats_[0].counted = params->awb.awb_mean[0].cnt;
	awbStats_[0].gSum = 4 * params->awb.awb_mean[0].mean_y_or_g * awbStats_->counted;
	awbStats_[0].bSum = 4 * params->awb.awb_mean[0].mean_cb_or_b * awbStats_->counted;
	awbStats_[0].rSum = 4 * params->awb.awb_mean[0].mean_cr_or_r * awbStats_->counted;
}

void RkISP1Awb::clearAwbStats()
{
	for (unsigned int i = 0; i < kAwbStatsSizeX * kAwbStatsSizeY; i++) {
		awbStats_[i].bSum = 0;
		awbStats_[i].rSum = 0;
		awbStats_[i].gSum = 0;
		awbStats_[i].counted = 0;
		awbStats_[i].uncounted = 0;
	}
}

void RkISP1Awb::calculateWBGains(const rkisp1_stat_buffer *stats)
{
	zones_.clear();
	clearAwbStats();
	generateAwbStats(stats);
	generateZones(zones_);
	LOG(RkISP1Awb, Debug) << "Valid zones: " << zones_.size();
	if (zones_.size() != 0) {
		awbGreyWorld(zones_, asyncResults_);
		LOG(RkISP1Awb, Error) << "Gain found for red: " << asyncResults_.redGain
				      << " and for blue: " << asyncResults_.blueGain;
	}
}

void RkISP1Awb::updateWbParameters(rkisp1_params_cfg &params)
{
	params.module_en_update |= RKISP1_CIF_ISP_MODULE_AWB | RKISP1_CIF_ISP_MODULE_AWB_GAIN;
	params.module_ens |= RKISP1_CIF_ISP_MODULE_AWB | RKISP1_CIF_ISP_MODULE_AWB_GAIN;
	params.meas.awb_meas_config.awb_mode = RKISP1_CIF_ISP_AWB_MODE_RGB;
	params.module_cfg_update |= RKISP1_CIF_ISP_MODULE_AWB_GAIN | RKISP1_CIF_ISP_MODULE_AWB;
	/*
	 * rkisp1_cif_isp_awb_gain_config
	 * All fields in this struct are 10 bit, where: 0x100h = 1
	 * unsigned integer value, range 0 to 4 with 8 bit fractional part.
	 * out_data_x = ( AWB_GAIN_X * in_data + 128) >> 8
	 */

	params.others.awb_gain_config.gain_green_b = 256;
	params.others.awb_gain_config.gain_blue = std::clamp(256 * asyncResults_.blueGain, 128.0, 512.0);
	params.others.awb_gain_config.gain_red = std::clamp(256 * asyncResults_.redGain, 128.0, 512.0);
	params.others.awb_gain_config.gain_green_r = 256;

	LOG(RkISP1Awb, Debug) << "Color temperature estimated: " << asyncResults_.temperatureK;
}

} /* namespace ipa::rkisp1 */

} /* namespace libcamera */
