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

	void calculateWBGains(Rectangle roi,
			const ipu3_uapi_stats_3a *stats);

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
	uint16_t wbGains_[4];
	uint8_t	rst_ae_hist_;
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

	const auto itGain = ctrls_.find(V4L2_CID_DIGITAL_GAIN);
	if (itGain == ctrls_.end()) {
		LOG(IPAIPU3, Error) << "Can't find gain control";
		return;
	}

	minGain_ = std::max<uint32_t>(itGain->second.min().get<int32_t>(), 1);
	maxGain_ = itGain->second.max().get<int32_t>();

	ControlList ctrls(ctrls_);
	exposure_ = minExposure_;
	gain_ = minGain_;

	memset(wbGains_, 0, sizeof(wbGains_));
	rst_ae_hist_ = 1;

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

const struct ipu3_uapi_bnr_static_config imgu_css_bnr_defaults = {
	{ 16, 16, 16, 16 },			/* wb_gains */
	{ 255, 255, 255, 255 },			/* wb_gains_thr */
	{ 0, 0, 8, 6, 0, 14 },			/* thr_coeffs */
	{ 0, 0, 0, 0 },				/* thr_ctrl_shd */
	{ -128, 0, -128, 0 },			/* opt_center */
	{					/* lut */
		{ 17, 23, 28, 32, 36, 39, 42, 45,
		  48, 51, 53, 55, 58, 60, 62, 64,
		  66, 68, 70, 72, 73, 75, 77, 78,
		  80, 82, 83, 85, 86, 88, 89, 90 }
	},
	{ 4, 0, 1, 8, 0, 8, 0, 8, 0 },		/* bp_ctrl */
	{ 8, 4, 4, 0, 8, 0, 1, 1, 1, 1, 0 },	/* dn_detect_ctrl */
	1920,
	{2663424, 1498176},
};

const struct ipu3_uapi_ae_grid_config imgu_css_ae_grid_defaults = {
	.width = 20,
	.height = 12,
	.block_width_log2 = 6,
	.block_height_log2 = 6,
	.reserved0 = 0,
	.ae_en = 1,
	.rst_hist_array = 0,
	.done_rst_hist_array = 0,
	.x_start = 0,
	.y_start = 0,
	.x_end = 1279,
	.y_end = 767,
};
/* settings for Auto Exposure color correction matrix */
const struct ipu3_uapi_ae_ccm imgu_css_ae_ccm_defaults = {
	256, 256, 256, 256,		/* gain_gr/r/b/gb */
	.mat = { 128, 0, 0, 0, 0, 128, 0, 0, 0, 0, 128, 0, 0, 0, 0, 128 },
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

void IPAIPU3::fillParams(unsigned int frame, ipu3_uapi_params *params,
			 [[maybe_unused]] const ControlList &controls)
{
	/* Prepare parameters buffer. */
	memset(params, 0, sizeof(*params));

	params->use.acc_awb = 1;
	params->acc_param.awb.config = imgu_css_awb_defaults;

	// Activate wb gain
	params->use.acc_bnr = 1;
	params->acc_param.bnr = imgu_css_bnr_defaults;
	params->acc_param.bnr.wb_gains.gr = wbGains_[0];
	params->acc_param.bnr.wb_gains.r  = wbGains_[1];
	params->acc_param.bnr.wb_gains.b  = wbGains_[2];
	params->acc_param.bnr.wb_gains.gb = wbGains_[3];

	static const struct ipu3_uapi_ae_weight_elem
			weight_def = { 1, 1, 1, 1, 1, 1, 1, 1 };
	params->use.acc_ae = 1;
	params->acc_param.ae.grid_cfg = imgu_css_ae_grid_defaults;

	params->acc_param.ae.grid_cfg.rst_hist_array = rst_ae_hist_;

	params->acc_param.ae.ae_ccm = imgu_css_ae_ccm_defaults;
	for (int i = 0; i < IPU3_UAPI_AE_WEIGHTS; i++)
		params->acc_param.ae.weights[i] = weight_def;

	params->use.obgrid_param = 1;
	params->obgrid_param.gr = 4096;
	params->obgrid_param.r = 4096;
	params->obgrid_param.b = 4096;
	params->obgrid_param.gb = 4096;

	IPAOperationData op;
	op.operation = IPU3_IPA_ACTION_PARAM_FILLED;

	queueFrameAction.emit(frame, op);

	/* \todo Calculate new values for exposure_ and gain_. */
	setControls(frame);
}

void IPAIPU3::calculateWBGains(Rectangle roi, const ipu3_uapi_stats_3a *stats)
{
	float Gr=0, R=0, B=0, Gb=0;
	Point topleft = roi.topLeft();
	uint32_t startY = (topleft.y / 16) * 160 * 8;
	uint32_t startX = (topleft.x / 8) * 8;
	uint32_t endX = startX + (roi.size().width / 8);

	for (uint32_t j = (topleft.y / 16) ; j < roi.size().height / 16 ; j++) {
		for (uint32_t i=startX+startY ; i < endX+startY ; i+=8) {
			Gr += stats->awb_raw_buffer.meta_data[i];
			R += stats->awb_raw_buffer.meta_data[i+1];
			B += stats->awb_raw_buffer.meta_data[i+2];
			Gb += stats->awb_raw_buffer.meta_data[i+3];
		}
	}
	Gr /= (1280*45)/8;
	R /= (1280*45)/8;
	B /= (1280*45)/8;
	Gb /= (1280*45)/8;
	
	float G = (Gr+Gb)/2;

	gain_ = 16*(64/G);

	wbGains_[0] = 8192*(R/Gr);
	wbGains_[1] = 8192*(G/R);
	wbGains_[2] = 8192*(G/B);
	wbGains_[3] = 8192*(B/Gb);
}

void IPAIPU3::parseStatistics(unsigned int frame,
			      [[maybe_unused]] const ipu3_uapi_stats_3a *stats)
{
	ControlList ctrls(controls::controls);

	/* \todo React to statistics and update internal state machine. */
	/* \todo Add meta-data information to ctrls. */

	if (frame == 16) {
		LOG(IPAIPU3, Error) << "Write AWB bins";
		std::string filename = "/tmp/stats_frame30.bin";

		int fd = open(filename.c_str(), O_CREAT | O_WRONLY | O_APPEND,
				S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH);
		::write(fd, &stats->awb_raw_buffer.meta_data[0], IPU3_UAPI_AWB_MAX_BUFFER_SIZE);
		close(fd);
	}

	calculateWBGains(Rectangle(260, 150, 776, 650), stats);

#if 0
	float X=(-0.14282)*(R)+(1.54924)*(G)+(-0.95641)*(B);
	float Y=(-0.32466)*(R)+(1.57837)*(G)+(-0.73191)*(B);
	float Z=(-0.68202)*(R)+(0.77073)*(G)+(0.56332)*(B);

	float x=X/(X+Y+Z);
	float y=Y/(X+Y+Z);

	float n=(x-0.3320)/(0.1858-y);
	float CCT=449*n*n*n+3525*n*n+6823.3*n+5520.33;
	LOG(IPAIPU3, Error) << "Color temperature estimation:" << CCT;
#endif
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
		exposure_ = 35000;
		ctrls.set(V4L2_CID_EXPOSURE, static_cast<int32_t>(exposure_));
		gain_ = 2500;
		ctrls.set(V4L2_CID_DIGITAL_GAIN, static_cast<int32_t>(gain_));
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
