/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Ideas On Board
 *
 * ipu3_awb.cpp - AWB control algorithm
 */
#include <numeric>
#include <unordered_map>

#include "libcamera/internal/log.h"

#include "ipu3_awb.h"

namespace libcamera {

namespace ipa {

LOG_DEFINE_CATEGORY(IPU3Awb)

static const struct ipu3_uapi_bnr_static_config imgu_css_bnr_defaults = {
	{ 16, 16, 16, 16 }, /* wb_gains */
	{ 255, 255, 255, 255 }, /* wb_gains_thr */
	{ 0, 0, 8, 6, 0, 14 }, /* thr_coeffs */
	{ 0, 0, 0, 0 }, /* thr_ctrl_shd */
	{ -648, 0, -366, 0 }, /* opt_center */
	{ /* lut */
	  { 17, 23, 28, 32, 36, 39, 42, 45,
	    48, 51, 53, 55, 58, 60, 62, 64,
	    66, 68, 70, 72, 73, 75, 77, 78,
	    80, 82, 83, 85, 86, 88, 89, 90 } },
	{ 4, 0, 1, 8, 0, 8, 0, 8, 0 }, /* bp_ctrl */
	{ 8, 4, 4, 0, 8, 0, 1, 1, 1, 1, 0 }, /* dn_detect_ctrl */
	1296,
	{ 419904, 133956 },
};

/* settings for Auto White Balance */
static const struct ipu3_uapi_awb_config_s imgu_css_awb_defaults = {
	8191,
	8191,
	8191,
	8191 | /* rgbs_thr_gr/r/gb/b */
		IPU3_UAPI_AWB_RGBS_THR_B_EN | IPU3_UAPI_AWB_RGBS_THR_B_INCL_SAT,
	.grid = {
		.width = 160,
		.height = 45,
		.block_width_log2 = 3,
		.block_height_log2 = 4,
		.x_start = 0,
		.y_start = 0,
	},
};

static const struct ipu3_uapi_ccm_mat_config imgu_css_ccm_3800k = {
	7379, -526, -296, 0,
	-411, 7397, -415, 0,
	-224, -564, 7244, 0
};

IPU3Awb::IPU3Awb()
	: Algorithm()
{
}

IPU3Awb::~IPU3Awb()
{
}

void IPU3Awb::initialise()
{
}

static void fillGamma(uint32_t x0, uint32_t y0,
								uint32_t x1, uint32_t y1,
								ipu3_uapi_params &params)
{
	uint32_t a = (y1 - y0) / (x1 - x0);
	uint32_t b = y1 - a * x1;
	for (uint32_t i = x0; i < x1; i++)
		params.acc_param.gamma.gc_lut.lut[i] = a * i + b;
}

void IPU3Awb::initialise(ipu3_uapi_params &params)
{
	params.use.acc_awb = 1;
	/*\todo fill the grid calculated based on BDS configuration */
	params.acc_param.awb.config = imgu_css_awb_defaults;

	params.use.acc_bnr = 1;
	params.acc_param.bnr = imgu_css_bnr_defaults;

	params.use.acc_ccm = 1;
	params.acc_param.ccm = imgu_css_ccm_3800k;

	params.use.acc_gamma = 1;
	params.acc_param.gamma.gc_ctrl.enable = 1;

	fillGamma(0, 0, 12, 33, params);
	fillGamma(12, 33, 30, 94, params);
	fillGamma(30, 94, 55, 245, params);
	fillGamma(55, 245, 90, 683, params);
	fillGamma(90, 683, 139, 2012, params);
	fillGamma(139, 2012, 208, 5371, params);
	fillGamma(208, 5371, 255, 8191, params);

	wbGains_[0] = 8192 * 0.8;
	wbGains_[1] = 8192;
	wbGains_[2] = 8192;
	wbGains_[3] = 8192 * 0.8;

	frame_count_ = 0;
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

void IPU3Awb::calculateWBGains(Rectangle roi, const ipu3_uapi_stats_3a *stats)
{
	std::vector<uint32_t> redValues, greenRedValues, greenBlueValues, blueValues;
	Point topleft = roi.topLeft();
	uint32_t startY = (topleft.y / 16) * 160 * 8;
	uint32_t startX = (topleft.x / 8) * 8;
	uint32_t endX = (startX + (roi.size().width / 8)) * 8;

	for (uint32_t j = (topleft.y / 16); j < (topleft.y / 16) + (roi.size().height / 16); j++) {
		for (uint32_t i = startX + startY; i < endX + startY; i += 8) {
			greenRedValues.push_back(stats->awb_raw_buffer.meta_data[i]);
			redValues.push_back(stats->awb_raw_buffer.meta_data[i + 1]);
			blueValues.push_back(stats->awb_raw_buffer.meta_data[i + 2]);
			greenBlueValues.push_back(stats->awb_raw_buffer.meta_data[i + 3]);
		}
	}

	std::sort(redValues.begin(), redValues.end());
	std::sort(greenRedValues.begin(), greenRedValues.end());
	std::sort(blueValues.begin(), blueValues.end());
	std::sort(greenBlueValues.begin(), greenBlueValues.end());

	double Grmed = greenRedValues[greenRedValues.size() / 2];
	double Rmed = redValues[redValues.size() / 2];
	double Bmed = blueValues[blueValues.size() / 2];
	double Gbmed = greenBlueValues[greenBlueValues.size() / 2];

	double Rgain = Grmed / Rmed;
	double Bgain = Gbmed / Bmed;
	LOG(IPU3Awb, Debug) << "max R, Gr, B, Gb: "
			    << redValues.back() << ","
			    << greenRedValues.back() << ","
			    << blueValues.back() << ","
			    << greenBlueValues.back();
	tint_ = ((Rmed / Grmed) + (Bmed / Gbmed)) / 2;

	/* \todo Those are corrections when light is really low
	 * it should be taken into account by AGC somehow */
	if ((Rgain >= 2) && (Bgain < 2)) {
		wbGains_[0] = 4096 * tint_;
		wbGains_[1] = 8192 * Rgain;
		wbGains_[2] = 4096 * Bgain;
		wbGains_[3] = 4096 * tint_;
	} else if ((Bgain >= 2) && (Rgain < 2)) {
		wbGains_[0] = 4096 * tint_;
		wbGains_[1] = 4096 * Rgain;
		wbGains_[2] = 8192 * Bgain;
		wbGains_[3] = 4096 * tint_;
	} else {
		wbGains_[0] = 8192 * tint_;
		wbGains_[1] = 8192 * Rgain;
		wbGains_[2] = 8192 * Bgain;
		wbGains_[3] = 8192 * tint_;
	}

	frame_count_++;

	cct_ = estimateCCT(Rmed, (Grmed + Gbmed) / 2, Bmed);
}

void IPU3Awb::updateWbParameters(ipu3_uapi_params &params)
{
	if ((wbGains_[0] == 0) || (wbGains_[1] == 0) || (wbGains_[2] == 0) || (wbGains_[3] == 0)) {
		LOG(IPU3Awb, Error) << "Gains can't be 0, check the stats";
	} else {
		params.acc_param.bnr.wb_gains.gr = wbGains_[0];
		params.acc_param.bnr.wb_gains.r = wbGains_[1];
		params.acc_param.bnr.wb_gains.b = wbGains_[2];
		params.acc_param.bnr.wb_gains.gb = wbGains_[3];
		params.acc_param.ccm = imgu_css_ccm_3800k;
	}
}

} /* namespace ipa */

} /* namespace libcamera */