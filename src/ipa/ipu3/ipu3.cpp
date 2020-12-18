/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * ipu3.cpp - IPU3 Image Processing Algorithms
 */

#include <algorithm>
#include <math.h>
#include <queue>
#include <stdint.h>
#include <string.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>

#include <linux/intel-ipu3.h>
#include <linux/v4l2-controls.h>

#include <libcamera/buffer.h>
#include <libcamera/control_ids.h>
#include <libcamera/ipa/ipa_interface.h>
#include <libcamera/ipa/ipa_module_info.h>
#include <libcamera/ipa/ipu3.h>
#include <libcamera/request.h>

#include <libipa/ipa_interface_wrapper.h>

#include "libcamera/internal/log.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(IPAIPU3)

class IPAIPU3 : public IPAInterface
{
public:
	int init([[maybe_unused]] const IPASettings &settings) override
	{
		return 0;
	}
	int start() override { return 0; }
	void stop() override {}

	void configure(const CameraSensorInfo &info,
		       const std::map<unsigned int, IPAStream> &streamConfig,
		       const std::map<unsigned int, const ControlInfoMap &> &entityControls,
		       const IPAOperationData &ipaConfig,
		       IPAOperationData *response) override;
	void mapBuffers(const std::vector<IPABuffer> &buffers) override;
	void unmapBuffers(const std::vector<unsigned int> &ids) override;
	void processEvent(const IPAOperationData &event) override;

private:
	void fillParams(unsigned int frame, ipu3_uapi_params *params,
			const ControlList &controls);

	void parseStatistics(unsigned int frame,
			     const ipu3_uapi_stats_3a *stats);

	void setControls(unsigned int frame);

	std::map<unsigned int, FrameBuffer> buffers_;
	std::map<unsigned int, void *> buffersMemory_;

	ControlInfoMap ctrls_;

	/* Camera sensor controls. */
	uint32_t exposure_;
	uint32_t minExposure_;
	uint32_t maxExposure_;
	uint32_t gain_;
	uint32_t minGain_;
	uint32_t maxGain_;
};

void IPAIPU3::configure([[maybe_unused]] const CameraSensorInfo &info,
			[[maybe_unused]] const std::map<unsigned int, IPAStream> &streamConfig,
			const std::map<unsigned int, const ControlInfoMap &> &entityControls,
			[[maybe_unused]] const IPAOperationData &ipaConfig,
			[[maybe_unused]] IPAOperationData *result)
{
	if (entityControls.empty())
		return;

	ctrls_ = entityControls.at(0);

	const auto itExp = ctrls_.find(V4L2_CID_EXPOSURE);
	if (itExp == ctrls_.end()) {
		LOG(IPAIPU3, Error) << "Can't find exposure control";
		return;
	}

	minExposure_ = std::max<uint32_t>(itExp->second.min().get<int32_t>(), 1);
	maxExposure_ = itExp->second.max().get<int32_t>();

	const auto itGain = ctrls_.find(V4L2_CID_GAIN);
	if (itGain == ctrls_.end()) {
		LOG(IPAIPU3, Error) << "Can't find gain control";
		return;
	}

	minGain_ = std::max<uint32_t>(itGain->second.min().get<int32_t>(), 1);
	maxGain_ = itGain->second.max().get<int32_t>();

	ControlList ctrls(ctrls_);
	exposure_ = minExposure_;
	gain_ = minGain_;

	setControls(0);
}

void IPAIPU3::mapBuffers(const std::vector<IPABuffer> &buffers)
{
	for (const IPABuffer &buffer : buffers) {
		auto elem = buffers_.emplace(std::piecewise_construct,
					     std::forward_as_tuple(buffer.id),
					     std::forward_as_tuple(buffer.planes));
		const FrameBuffer &fb = elem.first->second;

		buffersMemory_[buffer.id] = mmap(NULL,
						 fb.planes()[0].length,
						 PROT_READ | PROT_WRITE,
						 MAP_SHARED,
						 fb.planes()[0].fd.fd(),
						 0);

		if (buffersMemory_[buffer.id] == MAP_FAILED) {
			int ret = -errno;
			LOG(IPAIPU3, Fatal) << "Failed to mmap buffer: "
					    << strerror(-ret);
		}
	}
}

void IPAIPU3::unmapBuffers(const std::vector<unsigned int> &ids)
{
	for (unsigned int id : ids) {
		const auto fb = buffers_.find(id);
		if (fb == buffers_.end())
			continue;

		munmap(buffersMemory_[id], fb->second.planes()[0].length);
		buffersMemory_.erase(id);
		buffers_.erase(id);
	}
}

void IPAIPU3::processEvent(const IPAOperationData &event)
{
	switch (event.operation) {
	case IPU3_IPA_EVENT_PARSE_STAT: {
		unsigned int frame = event.data[0];
		unsigned int bufferId = event.data[1];

		struct ipu3_uapi_stats_3a stats;
		memcpy(&stats, buffersMemory_[bufferId], sizeof(stats));

		parseStatistics(frame, &stats);
		break;
	}
	case IPU3_IPA_EVENT_FILL_PARAMS: {
		unsigned int frame = event.data[0];
		unsigned int bufferId = event.data[1];

		ipu3_uapi_params *params =
			static_cast<ipu3_uapi_params *>(buffersMemory_[bufferId]);

		fillParams(frame, params, event.controls[0]);
		break;
	}
	default:
		LOG(IPAIPU3, Error) << "Unknown event " << event.operation;
		break;
	}
}

#define X 0	/*  Don't care value */
const struct ipu3_uapi_bnr_static_config imgu_css_bnr_defaults = {
	{ 16, 16, 16, 16 },			/* wb_gains */
	{ 255, 255, 255, 255 },			/* wb_gains_thr */
	{ 0, X, 8, 6, X, 14 },			/* thr_coeffs */
	{ 0, 0, 0, 0 },				/* thr_ctrl_shd */
	{ -128, X, -128, X },			/* opt_center */
	{					/* lut */
		{ 17, 23, 28, 32, 36, 39, 42, 45,
		  48, 51, 53, 55, 58, 60, 62, 64,
		  66, 68, 70, 72, 73, 75, 77, 78,
		  80, 82, 83, 85, 86, 88, 89, 90 }
	},
	{ 4, X, 1, 8, X, 8, X, 8, X },		/* bp_ctrl */
	{ 8, 4, 4, X, 8, X, 1, 1, 1, 1, X },	/* dn_detect_ctrl */
	1920,
	{2663424, 1498176},
};

const struct ipu3_uapi_gamma_corr_lut imgu_css_gamma_lut = { {
	32, 33, 35, 55, 80, 143, 159, 175, 191, 207, 223, 239, 255, 271, 287,
	303, 319, 335, 351, 367, 383, 399, 415, 431, 447, 463, 479, 495, 511,
	527, 543, 559, 575, 591, 607, 623, 639, 655, 671, 687, 703, 719, 735,
	751, 767, 783, 799, 815, 831, 847, 863, 879, 895, 911, 927, 943, 959,
	975, 991, 1007, 1023, 1039, 1055, 1071, 1087, 1103, 1119, 1135, 1151,
	1167, 1183, 1199, 1215, 1231, 1247, 1263, 1279, 1295, 1311, 1327, 1343,
	1359, 1375, 1391, 1407, 1423, 1439, 1455, 1471, 1487, 1503, 1519, 1535,
	1551, 1567, 1583, 1599, 1615, 1631, 1647, 1663, 1679, 1695, 1711, 1727,
	1743, 1759, 1775, 1791, 1807, 1823, 1839, 1855, 1871, 1887, 1903, 1919,
	1935, 1951, 1967, 1983, 1999, 2015, 2031, 2047, 2063, 2079, 2095, 2111,
	2143, 2175, 2207, 2239, 2271, 2303, 2335, 2367, 2399, 2431, 2463, 2495,
	2527, 2559, 2591, 2623, 2655, 2687, 2719, 2751, 2783, 2815, 2847, 2879,
	2911, 2943, 2975, 3007, 3039, 3071, 3103, 3135, 3167, 3199, 3231, 3263,
	3295, 3327, 3359, 3391, 3423, 3455, 3487, 3519, 3551, 3583, 3615, 3647,
	3679, 3711, 3743, 3775, 3807, 3839, 3871, 3903, 3935, 3967, 3999, 4031,
	4063, 4095, 4127, 4159, 4223, 4287, 4351, 4415, 4479, 4543, 4607, 4671,
	4735, 4799, 4863, 4927, 4991, 5055, 5119, 5183, 5247, 5311, 5375, 5439,
	5503, 5567, 5631, 5695, 5759, 5823, 5887, 5951, 6015, 6079, 6143, 6207,
	6271, 6335, 6399, 6463, 6527, 6591, 6655, 6719, 6783, 6847, 6911, 6975,
	7039, 7103, 7167, 7231, 7295, 7359, 7423, 7487, 7551, 7615, 7679, 7743,
	7807, 7871, 7935, 8191, 8191, 8191, 8191
} };
#if 1
const struct ipu3_uapi_ae_grid_config imgu_css_ae_grid_defaults = {
	.width = 16,
	.height = 16,
	.block_width_log2 = 7,
	.block_height_log2 = 7,
	.reserved0 = 0,
	.ae_en = 1,
	.rst_hist_array = 0,
	.done_rst_hist_array = 0,
	.x_start = 632,
	.y_start = 352,
	.x_end = 4096+648,
	.y_end = 4096+368,
};
/* settings for Auto Exposure color correction matrix */
const struct ipu3_uapi_ae_ccm imgu_css_ae_ccm_defaults = {
	256, 256, 256, 256,		/* gain_gr/r/b/gb */
	.mat = { 128, 0, 0, 0, 0, 128, 0, 0, 0, 0, 128, 0, 0, 0, 0, 128 },
};

#endif

/* settings for Auto White Balance */
const struct ipu3_uapi_awb_config_s imgu_css_awb_defaults = {
	0, 1, 1, 0|	/* rgbs_thr_gr/r/gb/b */
	IPU3_UAPI_AWB_RGBS_THR_B_EN | IPU3_UAPI_AWB_RGBS_THR_B_INCL_SAT,
	.grid = {
		.width = 16,
		.height = 16,
		.block_width_log2 = 3,
		.block_height_log2 = 3,
		.x_start = 16,
		.y_start = 0,
	},
};

void IPAIPU3::fillParams(unsigned int frame, ipu3_uapi_params *params,
			 [[maybe_unused]] const ControlList &controls)
{
	/* Prepare parameters buffer. */
	memset(params, 0, sizeof(*params));

	params->use.acc_awb = 1;
	params->acc_param.awb.config = imgu_css_awb_defaults;
	params->acc_param.awb.config.rgbs_thr_gr = frame;
	params->acc_param.awb.config.rgbs_thr_r = frame;
	params->acc_param.awb.config.rgbs_thr_gb = frame;
	params->acc_param.awb.config.rgbs_thr_b = (frame) | IPU3_UAPI_AWB_RGBS_THR_B_EN | IPU3_UAPI_AWB_RGBS_THR_B_INCL_SAT;
	params->acc_param.awb.config.grid.x_start = frame;
	LOG(IPAIPU3, Error) << "x_start: " << (int)params->acc_param.awb.config.grid.x_start;

	// Activate wb gain
	params->use.acc_bnr = 0;
	params->acc_param.bnr = imgu_css_bnr_defaults;
	// daylight defaults... somehow :-)
	//params->acc_param.bnr.wb_gains.gr=8191;
	//params->acc_param.bnr.wb_gains.r=16384*1.2;
	//params->acc_param.bnr.wb_gains.b=16384*0.8;
	//params->acc_param.bnr.wb_gains.gb=params->acc_param.bnr.wb_gains.gr;

	// Correct gamma
	params->use.acc_gamma = 0;
	params->acc_param.gamma.gc_ctrl.enable = 1;
	params->acc_param.gamma.gc_lut = imgu_css_gamma_lut;
#if 1
	static const struct ipu3_uapi_ae_weight_elem
			weight_def = { 1, 1, 1, 1, 1, 1, 1, 1 };
	params->use.acc_ae = 0;
	if (frame == 0) {
		params->acc_param.ae.grid_cfg.rst_hist_array = 1;
		params->acc_param.ae.grid_cfg.done_rst_hist_array = 1;
	} else {
		params->acc_param.ae.grid_cfg.rst_hist_array = 0;
		params->acc_param.ae.grid_cfg.done_rst_hist_array = 1;
	}
	params->acc_param.ae.grid_cfg = imgu_css_ae_grid_defaults;
	params->acc_param.ae.ae_ccm = imgu_css_ae_ccm_defaults;
	for (int i = 0; i < IPU3_UAPI_AE_WEIGHTS; i++)
		params->acc_param.ae.weights[i] = weight_def;
#endif
	IPAOperationData op;
	op.operation = IPU3_IPA_ACTION_PARAM_FILLED;

	queueFrameAction.emit(frame, op);

	/* \todo Calculate new values for exposure_ and gain_. */
	setControls(frame);
}

void IPAIPU3::parseStatistics(unsigned int frame,
			      [[maybe_unused]] const ipu3_uapi_stats_3a *stats)
{
	ControlList ctrls(controls::controls);

	/* \todo React to statistics and update internal state machine. */
	/* \todo Add meta-data information to ctrls. */

//	LOG(IPAIPU3, Error) << "awb_en: " << stats->stats_3a_status.awb_en;
//	LOG(IPAIPU3, Error) << "ae_en: " << stats->stats_3a_status.ae_en;
//	LOG(IPAIPU3, Error) << "af_en: " << stats->stats_3a_status.af_en;
//	LOG(IPAIPU3, Error) << "awb_fr_en: " << stats->stats_3a_status.awb_en;

	if (frame%8 == 0) {
		LOG(IPAIPU3, Error) << "x_start: " << (int)stats->stats_4a_config.awb_config.grid.x_start;
		LOG(IPAIPU3, Error) << "rgbs_thr_gr: " << (int)stats->stats_4a_config.awb_config.rgbs_thr_gr;
		LOG(IPAIPU3, Error) << "rgbs_thr_r: " << (int)stats->stats_4a_config.awb_config.rgbs_thr_r;
		LOG(IPAIPU3, Error) << "rgbs_thr_gb: " << (int)stats->stats_4a_config.awb_config.rgbs_thr_gb;
		LOG(IPAIPU3, Error) << "rgbs_thr_b: " << (int)stats->stats_4a_config.awb_config.rgbs_thr_b;
	}
		std::string filename = "/tmp/stats_frame30.bin";
		int fd = open(filename.c_str(), O_CREAT | O_WRONLY | O_APPEND,
				S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH);
		::write(fd, &stats->awb_raw_buffer.meta_data[0], IPU3_UAPI_AWB_MAX_BUFFER_SIZE);
		close(fd);
	if (frame == 0) {
		LOG(IPAIPU3, Error) << "width: " << (int)stats->stats_4a_config.awb_config.grid.width;
		LOG(IPAIPU3, Error) << "height: " << (int)stats->stats_4a_config.awb_config.grid.height;
		LOG(IPAIPU3, Error) << "block_width_log2: " << (int)stats->stats_4a_config.awb_config.grid.block_width_log2;
		LOG(IPAIPU3, Error) << "block_height_log2: " << (int)stats->stats_4a_config.awb_config.grid.block_height_log2;
		LOG(IPAIPU3, Error) << "height_per_slice: " << (int)stats->stats_4a_config.awb_config.grid.height_per_slice;
		LOG(IPAIPU3, Error) << "x_start: " << (int)stats->stats_4a_config.awb_config.grid.x_start;
		LOG(IPAIPU3, Error) << "y_start: " << (int)stats->stats_4a_config.awb_config.grid.y_start;
		LOG(IPAIPU3, Error) << "x_end: " << (int)stats->stats_4a_config.awb_config.grid.x_end;
		LOG(IPAIPU3, Error) << "y_end: " << (int)stats->stats_4a_config.awb_config.grid.y_end;
	}
#if 0
		if (stats->stats_3a_status.awb_en)
			for (int i = 0 ; i < IPU3_UAPI_AWB_MAX_BUFFER_SIZE; i++)
				LOG(IPAIPU3, Error) << "stat AWB [" << i << "]: " << (int)(stats->awb_raw_buffer.meta_data[i]);

		if (stats->stats_3a_status.ae_en)
			for (int i = 0 ; i < IPU3_UAPI_AE_BINS * IPU3_UAPI_AE_COLORS; i++)
				LOG(IPAIPU3, Error) << "stat AE [" << i << "]: " << stats->ae_raw_buffer[0].buff.vals[i];
#endif
	//}
	IPAOperationData op;
	op.operation = IPU3_IPA_ACTION_METADATA_READY;
	op.controls.push_back(ctrls);

	queueFrameAction.emit(frame, op);
}

void IPAIPU3::setControls(unsigned int frame)
{
	IPAOperationData op;
	op.operation = IPU3_IPA_ACTION_SET_SENSOR_CONTROLS;

	ControlList ctrls(ctrls_);

	if (frame == 0) {
		exposure_ = 35000*2;
		ctrls.set(V4L2_CID_EXPOSURE, static_cast<int32_t>(exposure_));
		gain_ = 128;
		ctrls.set(V4L2_CID_GAIN, static_cast<int32_t>(gain_));
	}
	op.controls.push_back(ctrls);

	queueFrameAction.emit(frame, op);
}

/*
 * External IPA module interface
 */

extern "C" {
const struct IPAModuleInfo ipaModuleInfo = {
	IPA_MODULE_API_VERSION,
	1,
	"PipelineHandlerIPU3",
	"ipu3",
};

struct ipa_context *ipaCreate()
{
	return new IPAInterfaceWrapper(std::make_unique<IPAIPU3>());
}
}

} /* namespace libcamera */
