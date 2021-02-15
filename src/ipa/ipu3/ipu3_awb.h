/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * awb.h - AWB control algorithm
 */
#ifndef __LIBCAMERA_IPU3_AWB_H__
#define __LIBCAMERA_IPU3_AWB_H__

#include <linux/intel-ipu3.h>

#include <libcamera/geometry.h>

#include <libcamera/ipa/awb_algorithm.h>
#include <libcamera/ipa/ipa_controller.h>

namespace libcamera {

const struct ipu3_uapi_bnr_static_config imgu_css_bnr_defaults = {
	{ 16, 16, 16, 16 },			/* wb_gains */
	{ 255, 255, 255, 255 },			/* wb_gains_thr */
	{ 0, 0, 8, 6, 0, 14 },			/* thr_coeffs */
	{ 0, 0, 0, 0 },				/* thr_ctrl_shd */
	{ -648, 0, -366, 0 },			/* opt_center */
	{					/* lut */
		{ 17, 23, 28, 32, 36, 39, 42, 45,
		  48, 51, 53, 55, 58, 60, 62, 64,
		  66, 68, 70, 72, 73, 75, 77, 78,
		  80, 82, 83, 85, 86, 88, 89, 90 }
	},
	{ 4, 0, 1, 8, 0, 8, 0, 8, 0 },		/* bp_ctrl */
	{ 8, 4, 4, 0, 8, 0, 1, 1, 1, 1, 0 },	/* dn_detect_ctrl */
	1296,
	{419904, 133956},
};

/* settings for Auto White Balance */
const struct ipu3_uapi_awb_config_s imgu_css_awb_defaults = {
	8191, 8191, 8191, 8191|	/* rgbs_thr_gr/r/gb/b */
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

const struct ipu3_uapi_ccm_mat_config imgu_css_ccm_6000k = {
	7239, -750, -37, 0,
	-215, 9176, -200, 0,
	-70, -589, 6810, 0
};

const struct ipu3_uapi_ccm_mat_config imgu_css_ccm_4900k = {
	7811, -464, -466, 0,
	-635, 8762, -533, 0,
	-469, -154, 6583, 0
};

const struct ipu3_uapi_ccm_mat_config imgu_css_ccm_3800k = {
	7379, -526, -296, 0,
	-411, 7397, -415, 0,
	-224, -564, 7244, 0
};

struct AwbConfig {
	// Only repeat the AWB calculation every "this many" frames
	uint16_t frame_period;
	// number of initial frames for which speed taken as 1.0 (maximum)
	uint16_t startup_frames;
	unsigned int convergence_frames; // approx number of frames to converge
	double speed; // IIR filter speed applied to algorithm results
};

#if 0
typedef struct awb_public_set_item{
    unsigned char Gr_avg;
    unsigned char R_avg;
    unsigned char B_avg;
    unsigned char Gb_avg;
    unsigned char sat_ratio;
    unsigned char padding0; /**< Added the padding so that the public matches that private */
    unsigned char padding1; /**< Added the padding so that the public matches that private */
    unsigned char padding2; /**< Added the padding so that the public matches that private */
} awb_public_set_item_t;
#endif

class IPU3Awb : public AwbAlgorithm
{
public:
	IPU3Awb(IPAController *controller = NULL);
	~IPU3Awb();
	virtual char const *Name() const override;
	virtual void Initialise() override;
	void Initialise(ipu3_uapi_params &params);
	unsigned int GetConvergenceFrames() const override;
	void SetMode(std::string const &name) override;
	void SetManualGains(double manual_r, double manual_b) override;
	void calculateWBGains(Rectangle roi,
                        const ipu3_uapi_stats_3a *stats);
	void updateBNR(ipu3_uapi_params &params);

private:
	uint32_t estimateCCT(uint8_t R, uint8_t G, uint8_t B);

	/* configuration is read-only, and available to both threads */
	AwbConfig config_;
	std::string mode_name_;
	/* manual r setting */
	double manual_r_;
	/* manual b setting */
	double manual_b_;
	/* WB calculated gains */
	uint16_t wbGains_[4];
	double tint_;
	uint32_t cct_;

	uint32_t frame_count_;

	bool algoConverged_;
};
} /* namespace libcamera */

#endif /* __LIBCAMERA_IPU3_AWB_H__ */
