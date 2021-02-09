/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * ipu3.cpp - IPU3 Image Processing Algorithms
 */

#include <libcamera/ipa/ipu3.h>

#include <stdint.h>
#include <sys/mman.h>

#include <vector>                                                               
#include <iostream>  
#include <numeric>                                                              

#include <linux/intel-ipu3.h>
#include <linux/v4l2-controls.h>

#include <libcamera/buffer.h>
#include <libcamera/control_ids.h>
#include <libcamera/ipa/ipa_interface.h>
#include <libcamera/ipa/ipa_module_info.h>
#include <libcamera/request.h>

#include <libipa/ipa_interface_wrapper.h>

#include "libcamera/internal/buffer.h"
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
	int start([[maybe_unused]] const IPAOperationData &data,
		  [[maybe_unused]] IPAOperationData *result) override { return 0; }
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

	float estimateCCT(float R, float G, float B);
	void vectorizeWBGains(const ipu3_uapi_stats_3a *stats);
	void calculateWBGains(Rectangle roi,
                        const ipu3_uapi_stats_3a *stats);

	void parseStatistics(unsigned int frame,
			     const ipu3_uapi_stats_3a *stats);

	void setControls(unsigned int frame);

	std::map<unsigned int, MappedFrameBuffer> buffers_;

	ControlInfoMap ctrls_;

	/* Camera sensor controls. */
	uint32_t exposure_;
	uint32_t minExposure_;
	uint32_t maxExposure_;
	uint32_t gain_;
	uint32_t minGain_;
	uint32_t maxGain_;
	uint16_t wbGains_[4];
	Rectangle metering_regions[15];
	uint8_t metering_weights[15];

	float brightness_;
	float colorTemp_;

	std::vector<double> brightnessVec_;
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

	const auto itGain = ctrls_.find(V4L2_CID_ANALOGUE_GAIN);
	if (itGain == ctrls_.end()) {
		LOG(IPAIPU3, Error) << "Can't find gain control";
		return;
	}

	minExposure_ = std::max(itExp->second.min().get<int32_t>(), 1);
	maxExposure_ = itExp->second.max().get<int32_t>();
	exposure_ = minExposure_;

	minGain_ = std::max(itGain->second.min().get<int32_t>(), 1);
	maxGain_ = itGain->second.max().get<int32_t>();
	gain_ = minGain_;

	gain_ = 16;
	exposure_ = 25000;

	memset(wbGains_, 0, sizeof(wbGains_));
	brightness_ = 0;
	colorTemp_ = 0;

	metering_regions[11] = Rectangle(0, 0, 260, 144);
	metering_regions[9] = Rectangle(260, 0, 1280-2*260, 144);
	metering_regions[12] = Rectangle(1280-260, 0, 260, 144);

	metering_regions[7] = Rectangle(0, 144, 260, 720-144);
	metering_regions[13] = Rectangle(0, 720-144, 260, 144);

	metering_regions[8] = Rectangle(1280-260, 144, 260, 720-144);
	metering_regions[14] = Rectangle(1280-260, 720-144, 260, 144);

	metering_regions[0] = Rectangle(600, 360-144/2, 80, 144);
	metering_regions[1] = Rectangle(600-80, 360-144/2, 80, 144);
	metering_regions[2] = Rectangle(600+80, 360-144/2, 80, 144);

	metering_regions[3] = Rectangle(600-80, 360-144/2-144, 3*80, 144);
	metering_regions[4] = Rectangle(600-80, 360+144/2, 3*80, 144);

	metering_regions[5] = Rectangle(260, 144, 260, 720-144);
	metering_regions[6] = Rectangle(1280-2*260, 144, 260, 720-144);

	metering_regions[10] = Rectangle(260, 720-144, 1280-2*260, 144);

	metering_weights[0]  = 3;
	metering_weights[1]  = 3;
	metering_weights[2]  = 3;
	metering_weights[3]  = 2;
	metering_weights[4]  = 2;
	metering_weights[5]  = 2;
	metering_weights[6]  = 2;
	metering_weights[7]  = 1;
	metering_weights[8]  = 1;
	metering_weights[9]  = 1;
	metering_weights[10] = 1;
	metering_weights[11] = 0;
	metering_weights[12] = 0;
	metering_weights[13] = 0;
	metering_weights[14] = 0;

	setControls(0);
}

void IPAIPU3::mapBuffers(const std::vector<IPABuffer> &buffers)
{
	for (const IPABuffer &buffer : buffers) {
		const FrameBuffer fb(buffer.planes);
		buffers_.emplace(buffer.id,
				 MappedFrameBuffer(&fb, PROT_READ | PROT_WRITE));
	}
}

void IPAIPU3::unmapBuffers(const std::vector<unsigned int> &ids)
{
	for (unsigned int id : ids) {
		auto it = buffers_.find(id);
		if (it == buffers_.end())
			continue;

		buffers_.erase(it);
	}
}

void IPAIPU3::processEvent(const IPAOperationData &event)
{
	switch (event.operation) {
	case IPU3_IPA_EVENT_STAT_READY: {
		unsigned int frame = event.data[0];
		unsigned int bufferId = event.data[1];

		auto it = buffers_.find(bufferId);
		if (it == buffers_.end()) {
			LOG(IPAIPU3, Error) << "Could not find stats buffer!";
			return;
		}

		Span<uint8_t> mem = it->second.maps()[0];
		const ipu3_uapi_stats_3a *stats =
			reinterpret_cast<ipu3_uapi_stats_3a *>(mem.data());

		parseStatistics(frame, stats);
		break;
	}
	case IPU3_IPA_EVENT_FILL_PARAMS: {
		unsigned int frame = event.data[0];
		unsigned int bufferId = event.data[1];

		auto it = buffers_.find(bufferId);
		if (it == buffers_.end()) {
			LOG(IPAIPU3, Error) << "Could not find param buffer!";
			return;
		}

		Span<uint8_t> mem = it->second.maps()[0];
		ipu3_uapi_params *params =
			reinterpret_cast<ipu3_uapi_params *>(mem.data());

		fillParams(frame, params, event.controls[0]);
		break;
	}
	default:
		LOG(IPAIPU3, Error) << "Unknown event " << event.operation;
		break;
	}
}
#if 0
const struct ipu3_uapi_ccm_mat_config imgu_css_ccm_defaults = {
       15553, -6852, -509, 0,
       -3210, 13918, -2516, 0,
       -867, -6608, 15667, 0
};
#endif

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

	params->use.acc_ccm = 1;
	if (colorTemp_ > 4900) {
		params->acc_param.ccm = imgu_css_ccm_4900k;
		params->acc_param.bnr.wb_gains.gr = wbGains_[0];
		params->acc_param.bnr.wb_gains.r  = wbGains_[1];
		params->acc_param.bnr.wb_gains.b  = wbGains_[2];
		params->acc_param.bnr.wb_gains.gb = wbGains_[3];
	}
	else {
		params->acc_param.ccm = imgu_css_ccm_3800k;
		params->acc_param.bnr.wb_gains.gr = 8192*0.6307;
		params->acc_param.bnr.wb_gains.r  = 8192*1.283;
		params->acc_param.bnr.wb_gains.b  = 8192*1.888;
		params->acc_param.bnr.wb_gains.gb = 8192*0.6307;
	}
#if 0
	else {
		params->acc_param.ccm = imgu_css_ccm_defaults;
		params->acc_param.ccm = imgu_css_ccm_3800k;
		params->acc_param.bnr.wb_gains.gr = wbGains_[0];
		params->acc_param.bnr.wb_gains.r  = wbGains_[1];
		params->acc_param.bnr.wb_gains.b  = wbGains_[2];
		params->acc_param.bnr.wb_gains.gb = wbGains_[3];
	}
#endif
	IPAOperationData op;
	op.operation = IPU3_IPA_ACTION_PARAM_FILLED;

	queueFrameAction.emit(frame, op);

	/* \todo Calculate new values for exposure_ and gain_. */
	setControls(frame);
}

float IPAIPU3::estimateCCT(float R, float G, float B)
{
	float X=(-0.14282)*(R)+(1.54924)*(G)+(-0.95641)*(B);
	float Y=(-0.32466)*(R)+(1.57837)*(G)+(-0.73191)*(B);
	float Z=(-0.68202)*(R)+(0.77073)*(G)+(0.56332)*(B);

	float x=X/(X+Y+Z);
	float y=Y/(X+Y+Z);

	float n=(x-0.3320)/(0.1858-y);
	return 449*n*n*n+3525*n*n+6823.3*n+5520.33;
}

void IPAIPU3::vectorizeWBGains(const ipu3_uapi_stats_3a *stats)
{
	double Gr=0, R=0, B=0, Gb=0, G=0;
	brightnessVec_.clear();

	for (uint32_t j = 0 ; j < imgu_css_awb_defaults.grid.height ; j++) {
		for (uint32_t i = 0 ; i < imgu_css_awb_defaults.grid.width*8 ; i+=8) {
			Gr = stats->awb_raw_buffer.meta_data[i];
			R = stats->awb_raw_buffer.meta_data[i+1];
			B = stats->awb_raw_buffer.meta_data[i+2];
			Gb = stats->awb_raw_buffer.meta_data[i+3];
			G = (Gr+Gb)/2;
			brightnessVec_.push_back(0.299*R + 0.587*G + 0.114*B);
		}
	}
}

void IPAIPU3::calculateWBGains(Rectangle roi, const ipu3_uapi_stats_3a *stats)
{
	float Gr=0, R=0, B=0, Gb=0;
	Point topleft = roi.topLeft();
	uint32_t startY = (topleft.y / 16) * 160 * 8;
	uint32_t startX = (topleft.x / 8) * 8;
	uint32_t endX = (startX + (roi.size().width / 8))*8;
	uint32_t count = 0;

	for (uint32_t j = (topleft.y / 16) ; j < (topleft.y / 16)+(roi.size().height / 16) ; j++) {
		for (uint32_t i=startX+startY ; i < endX+startY ; i+=8) {
			Gr += stats->awb_raw_buffer.meta_data[i];
			R += stats->awb_raw_buffer.meta_data[i+1];
			B += stats->awb_raw_buffer.meta_data[i+2];
			Gb += stats->awb_raw_buffer.meta_data[i+3];
			count++;
		}
	}
	Gr /= count;
	R  /= count;
	B  /= count;
	Gb /= count;

	float G = (Gr+Gb)/2;

	colorTemp_ = estimateCCT(R, G, B);
	brightness_ = 0.299*R + 0.587*G + 0.114*B;

	float tint = 1/((G/R)+(G/B)/2);
	wbGains_[0] = 8192*tint;
	wbGains_[1] = 8192*(G/R);
	wbGains_[2] = 8192*(G/B);
	wbGains_[3] = 8192*tint;
}

void IPAIPU3::parseStatistics(unsigned int frame,
			      [[maybe_unused]] const ipu3_uapi_stats_3a *stats)
{
	ControlList ctrls(controls::controls);
	float brightness = 0;
	uint32_t sum_weights = 0;

	for (uint32_t i = 0; i < 15 ; i++) {
		calculateWBGains(metering_regions[i], stats);
		brightness += brightness_ * metering_weights[i];
		sum_weights += metering_weights[i];
	}
	brightness_ = brightness / sum_weights;
//	LOG(IPAIPU3, Error) << "["<<frame<<","<<gain_<<","<<exposure_<<"]"<<"Gain needed: " << 128/brightness_ << " Color temp estimation: " << colorTemp_;
	calculateWBGains(Rectangle(0, 0, 1280, 720), stats);

	vectorizeWBGains(stats);

	std::nth_element(brightnessVec_.begin(), brightnessVec_.begin() + brightnessVec_.size()/2, brightnessVec_.end());
	const auto [min, max] = std::minmax_element(begin(brightnessVec_), end(brightnessVec_));
	std::cout << "[" << gain_ << ", " << exposure_ << "] mean: " << std::accumulate( brightnessVec_.begin(), brightnessVec_.end(), 0.0)/brightnessVec_.size()
		<< " median: " << brightnessVec_[brightnessVec_.size()/2] << " min: " << *min << " max: " << *max << std::endl;
 
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
		ctrls.set(V4L2_CID_EXPOSURE, static_cast<int32_t>(exposure_));
		ctrls.set(V4L2_CID_ANALOGUE_GAIN, static_cast<int32_t>(gain_));
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
