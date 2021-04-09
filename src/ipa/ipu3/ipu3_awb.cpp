/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Ideas On Board
 *
 * ipu3_awb.cpp - AWB control algorithm
 */
#include "ipu3_awb.h"

#include <cmath>
#include <numeric>
#include <unordered_map>

#include "libcamera/internal/log.h"

namespace libcamera {

namespace ipa {

LOG_DEFINE_CATEGORY(IPU3Awb)

static const struct ipu3_uapi_bnr_static_config imguCssBnrDefaults = {
	.wb_gains = { 16, 16, 16, 16 },
	.wb_gains_thr = { 255, 255, 255, 255 },
	.thr_coeffs = { 1700, 0, 31, 31, 0, 16 },
	.thr_ctrl_shd = { 26, 26, 26, 26 },
	.opt_center{ -648, 0, -366, 0 },
	.lut = {
		{ 17, 23, 28, 32, 36, 39, 42, 45,
		  48, 51, 53, 55, 58, 60, 62, 64,
		  66, 68, 70, 72, 73, 75, 77, 78,
		  80, 82, 83, 85, 86, 88, 89, 90 } },
	.bp_ctrl = { 20, 0, 1, 40, 0, 6, 0, 6, 0 },
	.dn_detect_ctrl{ 9, 3, 4, 0, 8, 0, 1, 1, 1, 1, 0 },
	.column_size = 1296,
	.opt_center_sqr = { 419904, 133956 },
};

/* settings for Auto White Balance */
static const struct ipu3_uapi_awb_config_s imguCssAwbDefaults = {
	.rgbs_thr_gr = 8191,
	.rgbs_thr_r = 8191,
	.rgbs_thr_gb = 8191,
	.rgbs_thr_b = 8191 | IPU3_UAPI_AWB_RGBS_THR_B_EN | IPU3_UAPI_AWB_RGBS_THR_B_INCL_SAT,
	.grid = {
		.width = 160,
		.height = 36,
		.block_width_log2 = 3,
		.block_height_log2 = 4,
		.height_per_slice = 1, /* Overridden by kernel. */
		.x_start = 0,
		.y_start = 0,
		.x_end = 0,
		.y_end = 0,
	},
};

static const struct ipu3_uapi_ccm_mat_config imguCssCcmDefault = {
	8191, 0, 0, 0,
	0, 8191, 0, 0,
	0, 0, 8191, 0
};

IPU3Awb::IPU3Awb()
	: Algorithm()
{
}

IPU3Awb::~IPU3Awb()
{
}

void IPU3Awb::initialise(ipu3_uapi_params &params, const Size &bdsOutputSize, struct ipu3_uapi_grid_config &bdsGrid)
{
	params.use.acc_awb = 1;
	params.acc_param.awb.config = imguCssAwbDefaults;

	awbGrid_ = bdsGrid;
	params.acc_param.awb.config.grid = awbGrid_;

	params.use.obgrid = 0;
	params.obgrid_param.gr = 20;
	params.obgrid_param.r = 28;
	params.obgrid_param.b = 28;
	params.obgrid_param.gb = 20;

	params.use.acc_bnr = 1;
	params.acc_param.bnr = imguCssBnrDefaults;
	params.acc_param.bnr.opt_center.x_reset = -1 * (bdsOutputSize.width / 2);
	params.acc_param.bnr.opt_center.y_reset = -1 * (bdsOutputSize.height / 2);
	params.acc_param.bnr.column_size = bdsOutputSize.width;
	params.acc_param.bnr.opt_center_sqr.x_sqr_reset = params.acc_param.bnr.opt_center.x_reset * params.acc_param.bnr.opt_center.x_reset;
	params.acc_param.bnr.opt_center_sqr.y_sqr_reset = params.acc_param.bnr.opt_center.y_reset * params.acc_param.bnr.opt_center.y_reset;

	params.use.acc_ccm = 1;
	params.acc_param.ccm = imguCssCcmDefault;

	params.use.acc_gamma = 1;
	params.acc_param.gamma.gc_ctrl.enable = 1;

	params.use.acc_green_disparity = 0;
	params.acc_param.green_disparity.gd_black = 2440;
	params.acc_param.green_disparity.gd_red = 4;
	params.acc_param.green_disparity.gd_blue = 4;
	params.acc_param.green_disparity.gd_green = 4;
	params.acc_param.green_disparity.gd_shading = 24;
	params.acc_param.green_disparity.gd_support = 2;
	params.acc_param.green_disparity.gd_clip = 1;
	params.acc_param.green_disparity.gd_central_weight = 5;

	params.use.acc_cds = 1;
	params.acc_param.cds.csc_en = 1;
	params.acc_param.cds.uv_bin_output = 0;
	params.acc_param.cds.ds_c00 = 0;
	params.acc_param.cds.ds_c01 = 1;
	params.acc_param.cds.ds_c02 = 1;
	params.acc_param.cds.ds_c03 = 0;
	params.acc_param.cds.ds_c10 = 0;
	params.acc_param.cds.ds_c11 = 1;
	params.acc_param.cds.ds_c12 = 1;
	params.acc_param.cds.ds_c13 = 0;
	params.acc_param.cds.ds_nf = 2;

	wbGains_[0] = 4096;
	wbGains_[1] = 4096;
	wbGains_[2] = 4096;
	wbGains_[3] = 4096;

	frame_count_ = 0;
	zones_.reserve(kAwbStatsSizeX * kAwbStatsSizeY);
}

uint32_t IPU3Awb::estimateCCT(uint8_t red, uint8_t green, uint8_t blue)
{
	double X = (-0.14282) * (red) + (1.54924) * (green) + (-0.95641) * (blue);
	double Y = (-0.32466) * (red) + (1.57837) * (green) + (-0.73191) * (blue);
	double Z = (-0.68202) * (red) + (0.77073) * (green) + (0.56332) * (blue);

	double x = X / (X + Y + Z);
	double y = Y / (X + Y + Z);

	double n = (x - 0.3320) / (0.1858 - y);
	return static_cast<uint32_t>(449 * n * n * n + 3525 * n * n + 6823.3 * n + 5520.33);
}

void IPU3Awb::generateZones(std::vector<RGB> &zones)
{
	for (unsigned int i = 0; i < kAwbStatsSizeX * kAwbStatsSizeY; i++) {
		RGB zone;
		double counted = awbStats_[i].counted;
		if (counted >= 16) {
			zone.G = awbStats_[i].gSum / counted;
			if (zone.G >= 32) {
				zone.R = awbStats_[i].rSum / counted;
				zone.B = awbStats_[i].bSum / counted;
				zones.push_back(zone);
			}
		}
	}
}

void IPU3Awb::generateAwbStats(const ipu3_uapi_stats_3a *stats)
{
	uint32_t regionWidth = round(awbGrid_.width / static_cast<double>(kAwbStatsSizeX));
	uint32_t regionHeight = round(awbGrid_.height / static_cast<double>(kAwbStatsSizeY));

	for (unsigned int j = 0; j < kAwbStatsSizeY * regionHeight; j++) {
		for (unsigned int i = 0; i < kAwbStatsSizeX * regionWidth; i++) {
			uint32_t cellPosition = j * awbGrid_.width + i;
			uint32_t cellX = (cellPosition / regionWidth) % kAwbStatsSizeX;
			uint32_t cellY = ((cellPosition / awbGrid_.width) / regionHeight) % kAwbStatsSizeY;

			uint32_t awbRegionPosition = cellY * kAwbStatsSizeX + cellX;
			cellPosition *= 8;
			if (stats->awb_raw_buffer.meta_data[cellPosition + 4] == 0) {
				/* The cell is not saturated */
				awbStats_[awbRegionPosition].counted++;
				uint32_t greenValue = stats->awb_raw_buffer.meta_data[cellPosition + 0] + stats->awb_raw_buffer.meta_data[cellPosition + 3];
				awbStats_[awbRegionPosition].gSum += greenValue / 2;
				awbStats_[awbRegionPosition].rSum += stats->awb_raw_buffer.meta_data[cellPosition + 1];
				awbStats_[awbRegionPosition].bSum += stats->awb_raw_buffer.meta_data[cellPosition + 2];
			}
		}
	}
}

void IPU3Awb::clearAwbStats()
{
	for (unsigned int i = 0; i < kAwbStatsSizeX * kAwbStatsSizeY; i++) {
		awbStats_[i].bSum = 0;
		awbStats_[i].rSum = 0;
		awbStats_[i].gSum = 0;
		awbStats_[i].counted = 0;
		awbStats_[i].notcounted = 0;
	}
	asyncResults_.redGain = 1.0;
	asyncResults_.blueGain = 1.0;
	asyncResults_.greenGain = 1.0;
}

void IPU3Awb::awbGrey()
{
	LOG(IPU3Awb, Debug) << "Grey world AWB";
	/**
	 * Make a separate list of the derivatives for each of red and blue, so
	 * that we can sort them to exclude the extreme gains.  We could
	 * consider some variations, such as normalising all the zones first, or
	 * doing an L2 average etc.
	 */
	std::vector<RGB> &redDerivative(zones_);
	std::vector<RGB> blueDerivative(redDerivative);
	std::sort(redDerivative.begin(), redDerivative.end(),
		  [](RGB const &a, RGB const &b) {
			  return a.G * b.R < b.G * a.R;
		  });
	std::sort(blueDerivative.begin(), blueDerivative.end(),
		  [](RGB const &a, RGB const &b) {
			  return a.G * b.B < b.G * a.B;
		  });

	/* Average the middle half of the values. */
	int discard = redDerivative.size() / 4;
	RGB sumRed(0, 0, 0), sumBlue(0, 0, 0);
	for (auto ri = redDerivative.begin() + discard,
		  bi = blueDerivative.begin() + discard;
	     ri != redDerivative.end() - discard; ri++, bi++)
		sumRed += *ri, sumBlue += *bi;

	double redGain = sumRed.G / (sumRed.R + 1),
	       blueGain = sumBlue.G / (sumBlue.B + 1);

	/* Color temperature is not relevant in Gray world */
	asyncResults_.temperature_K = 4500;
	asyncResults_.redGain = redGain;
	asyncResults_.greenGain = 1.0;
	asyncResults_.blueGain = blueGain;
}

void IPU3Awb::calculateWBGains(const ipu3_uapi_stats_3a *stats)
{
	ASSERT(stats->stats_3a_status.awb_en);
	zones_.clear();
	clearAwbStats();
	generateAwbStats(stats);
	generateZones(zones_);
	LOG(IPU3Awb, Debug) << "Valid zones: " << zones_.size();
	if (zones_.size() > 10)
		awbGrey();

	LOG(IPU3Awb, Debug) << "Gain found for red: " << asyncResults_.redGain
			    << " and for blue: " << asyncResults_.blueGain;

	wbGains_[0] = 1024 * asyncResults_.greenGain;
	wbGains_[1] = 4096 * asyncResults_.redGain;
	wbGains_[2] = 4096 * asyncResults_.blueGain;
	wbGains_[3] = 1024 * asyncResults_.greenGain;

	frame_count_++;
}

void IPU3Awb::updateWbParameters(ipu3_uapi_params &params, double agcGamma)
{
	params.acc_param.bnr.wb_gains.gr = wbGains_[0];
	params.acc_param.bnr.wb_gains.r = wbGains_[1];
	params.acc_param.bnr.wb_gains.b = wbGains_[2];
	params.acc_param.bnr.wb_gains.gb = wbGains_[3];

	LOG(IPU3Awb, Debug) << "Color temperature estimated: " << cct_
			    << " and gamma calculated: " << agcGamma;
	params.acc_param.ccm = imguCssCcmDefault;

	for (uint32_t i = 0; i < 256; i++) {
		double j = i / 255.0;
		double gamma = std::pow(j, 1.0 / agcGamma);
		params.acc_param.gamma.gc_lut.lut[i] = gamma * 8191;
	}
}

} /* namespace ipa */

} /* namespace libcamera */
