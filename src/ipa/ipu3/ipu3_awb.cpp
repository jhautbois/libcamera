/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * ipu3_awb.cpp - AWB control algorithm
 */
#include <iostream>
#include <numeric>
#include <unordered_map>

#include "libcamera/internal/log.h"

#include "ipu3_awb.h"

using namespace libcamera;

LOG_DEFINE_CATEGORY(IPU3Awb)

#define NAME "ipu3.awb"

IPU3Awb::IPU3Awb(IPAController *controller)
	: AwbAlgorithm(controller)
{
}

IPU3Awb::~IPU3Awb()
{
}

char const *IPU3Awb::Name() const
{
	return NAME;
}

void IPU3Awb::Initialise() {}

void IPU3Awb::Initialise(ipu3_uapi_params &params)
{
	memset(&params, 0, sizeof(params));
	memset(wbGains_, 8192, sizeof(wbGains_));
	wbGains_[0] = 8192 * 0.8;
	wbGains_[3] = 8192 * 0.8;
	params.use.acc_awb = 1;
	/*\todo fill the grid calculated based on BDS configuration */
	params.acc_param.awb.config = imgu_css_awb_defaults;

	params.use.acc_bnr = 1;
	params.acc_param.bnr = imgu_css_bnr_defaults;

	params.use.acc_ccm = 1;
	params.acc_param.ccm = imgu_css_ccm_3800k;

	params.use.acc_gamma = 1;
	params.acc_param.gamma.gc_ctrl.enable = 1;

	uint32_t a = (32 * 245) / (245 - 9);

	for (uint32_t i = 0; i < 10; i++)
		params.acc_param.gamma.gc_lut.lut[i] = 0;
	for (uint32_t i = 10; i < 245; i++)
		params.acc_param.gamma.gc_lut.lut[i] = a * i + (0 - a * 9);
	for (uint32_t i = 245; i < 255; i++)
		params.acc_param.gamma.gc_lut.lut[i] = 32 * 245;

	frame_count_ = 0;
	algoConverged_ = false;
}

unsigned int IPU3Awb::GetConvergenceFrames() const
{
	// If colour gains have been explicitly set, there is no convergence
	// to happen, so no need to drop any frames - return zero.
	if (manual_r_ && manual_b_)
		return 0;
	else
		return config_.convergence_frames;
}

void IPU3Awb::SetMode(std::string const &mode_name)
{
	mode_name_ = mode_name;
}

void IPU3Awb::SetManualGains(double manual_r, double manual_b)
{
	// If any of these are 0.0, we swich back to auto.
	manual_r_ = manual_r;
	manual_b_ = manual_b;
}

uint32_t IPU3Awb::estimateCCT(uint8_t R, uint8_t G, uint8_t B)
{
	double X = (-0.14282) * (R) + (1.54924) * (G) + (-0.95641) * (B);
	double Y = (-0.32466) * (R) + (1.57837) * (G) + (-0.73191) * (B);
	double Z = (-0.68202) * (R) + (0.77073) * (G) + (0.56332) * (B);

	double x = X / (X + Y + Z);
	double y = Y / (X + Y + Z);

	double n = (x - 0.3320) / (0.1858 - y);
	return static_cast<uint32_t>(449 * n * n * n + 3525 * n * n + 6823.3 * n + 5520.33);
}

void IPU3Awb::calculateWBGains([[maybe_unused]] Rectangle roi, const ipu3_uapi_stats_3a *stats)
{
	if (algoConverged_)
		return;

	std::vector<uint32_t> R_v, Gr_v, Gb_v, B_v;
	Point topleft = roi.topLeft();
	uint32_t startY = (topleft.y / 16) * 160 * 8;
	uint32_t startX = (topleft.x / 8) * 8;
	uint32_t endX = (startX + (roi.size().width / 8)) * 8;

	for (uint32_t j = (topleft.y / 16); j < (topleft.y / 16) + (roi.size().height / 16); j++) {
		for (uint32_t i = startX + startY; i < endX + startY; i += 8) {
			Gr_v.push_back(stats->awb_raw_buffer.meta_data[i]);
			R_v.push_back(stats->awb_raw_buffer.meta_data[i + 1]);
			B_v.push_back(stats->awb_raw_buffer.meta_data[i + 2]);
			Gb_v.push_back(stats->awb_raw_buffer.meta_data[i + 3]);
		}
	}

	std::sort(R_v.begin(), R_v.end());
	std::sort(Gr_v.begin(), Gr_v.end());
	std::sort(B_v.begin(), B_v.end());
	std::sort(Gb_v.begin(), Gb_v.end());

	double Grmed = Gr_v[Gr_v.size() / 2];
	double Rmed = R_v[R_v.size() / 2];
	double Bmed = B_v[B_v.size() / 2];
	double Gbmed = Gb_v[Gb_v.size() / 2];

	double Rgain = Grmed / Rmed;
	double Bgain = Gbmed / Bmed;
	LOG(IPU3Awb, Debug) << "max R, Gr, B, Gb: "
			    << R_v.back() << ","
			    << Gr_v.back() << ","
			    << B_v.back() << ","
			    << Gb_v.back();
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

	algoConverged_ = true;
}

void IPU3Awb::updateBNR(ipu3_uapi_params &params)
{
	if (!algoConverged_)
		return;

	params.acc_param.bnr.wb_gains.gr = wbGains_[0];
	params.acc_param.bnr.wb_gains.r = wbGains_[1];
	params.acc_param.bnr.wb_gains.b = wbGains_[2];
	params.acc_param.bnr.wb_gains.gb = wbGains_[3];
	if (cct_ < 5500)
		params.acc_param.ccm = imgu_css_ccm_3800k;
	else
		params.acc_param.ccm = imgu_css_ccm_6000k;
}
