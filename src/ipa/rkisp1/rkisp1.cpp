/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * rkisp1.cpp - RkISP1 Image Processing Algorithms
 */

#include <algorithm>
#include <math.h>
#include <queue>
#include <stdint.h>
#include <string.h>
#include <sys/mman.h>

#include <linux/rkisp1-config.h>
#include <linux/v4l2-controls.h>

#include <libcamera/base/log.h>

#include <libcamera/buffer.h>
#include <libcamera/control_ids.h>
#include <libcamera/ipa/ipa_interface.h>
#include <libcamera/ipa/ipa_module_info.h>
#include <libcamera/ipa/rkisp1_ipa_interface.h>
#include <libcamera/request.h>

#include "rkisp1_awb.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(IPARkISP1)

namespace ipa::rkisp1 {

class IPARkISP1 : public IPARkISP1Interface
{
public:
	int init(unsigned int hwRevision) override;
	int start() override;
	void stop() override {}

	int configure(const IPACameraSensorInfo &info,
		      const std::map<uint32_t, IPAStream> &streamConfig,
		      const std::map<uint32_t, ControlInfoMap> &entityControls) override;
	void mapBuffers(const std::vector<IPABuffer> &buffers) override;
	void unmapBuffers(const std::vector<unsigned int> &ids) override;
	void processEvent(const RkISP1Event &event) override;

private:
	void queueRequest(unsigned int frame, rkisp1_params_cfg *params,
			  const ControlList &controls);
	void updateStatistics(unsigned int frame,
			      const rkisp1_stat_buffer *stats);

	void setControls(unsigned int frame);
	void metadataReady(unsigned int frame, unsigned int aeState);
	void configureParams();

	std::map<unsigned int, FrameBuffer> buffers_;
	std::map<unsigned int, void *> buffersMemory_;

	ControlInfoMap ctrls_;

	/* Camera sensor controls. */
	bool autoExposure_;
	uint32_t exposure_;
	uint32_t minExposure_;
	uint32_t maxExposure_;
	uint32_t gain_;
	uint32_t minGain_;
	uint32_t maxGain_;

	/* Interface to the AWB algorithm */
	std::unique_ptr<RkISP1Awb> awbAlgo_;

	/* Local parameter storage */
	struct rkisp1_params_cfg params_;
	bool aeLocked_;
	bool awbLocked_;
};

int IPARkISP1::init(unsigned int hwRevision)
{
	/* \todo Add support for other revisions */
	if (hwRevision != RKISP1_V10) {
		LOG(IPARkISP1, Error)
			<< "Hardware revision " << hwRevision
			<< " is currently not supported";
		return -ENODEV;
	}

	LOG(IPARkISP1, Debug) << "Hardware revision is " << hwRevision;
	return 0;
}

int IPARkISP1::start()
{
	setControls(0);

	return 0;
}

static void configureAwb(struct rkisp1_params_cfg &params)
{
	params.module_cfg_update |= RKISP1_CIF_ISP_MODULE_AWB;
	params.module_en_update |= RKISP1_CIF_ISP_MODULE_AWB;
	params.module_ens |= RKISP1_CIF_ISP_MODULE_AWB;

	params.meas.awb_meas_config.awb_mode = RKISP1_CIF_ISP_AWB_MODE_RGB;
	params.meas.awb_meas_config.awb_wnd.h_offs = 1640 / 4;
	params.meas.awb_meas_config.awb_wnd.h_size = 1640 / 2;
	params.meas.awb_meas_config.awb_wnd.v_offs = 1232 / 4;
	params.meas.awb_meas_config.awb_wnd.v_size = 1232 / 2;

	params.meas.awb_meas_config.max_y = 230;
	params.meas.awb_meas_config.min_y = 250; // max_g
	params.meas.awb_meas_config.max_csum = 250;
	params.meas.awb_meas_config.min_c = 230;
	params.meas.awb_meas_config.awb_ref_cb = 16; // max b
	params.meas.awb_meas_config.awb_ref_cr = 16; // max r
	params.meas.awb_meas_config.enable_ymax_cmp = 0;
	params.meas.awb_meas_config.frames = 0;
}

static void configureAwbGains(struct rkisp1_params_cfg &params)
{
	params.module_en_update |= RKISP1_CIF_ISP_MODULE_AWB_GAIN;
	params.module_ens |= 0;
	params.module_cfg_update |= RKISP1_CIF_ISP_MODULE_AWB_GAIN;

	params.others.awb_gain_config.gain_green_b = 256;
	params.others.awb_gain_config.gain_blue = 256;
	params.others.awb_gain_config.gain_red = 256;
	params.others.awb_gain_config.gain_green_r = 256;
}

static void configureCtk(struct rkisp1_params_cfg &params)
{
	params.module_en_update |= RKISP1_CIF_ISP_MODULE_CTK;
	params.module_ens |= 0;
	params.module_cfg_update |= RKISP1_CIF_ISP_MODULE_CTK;
}

static void configureLsc(struct rkisp1_params_cfg &params)
{
	params.module_en_update |= RKISP1_CIF_ISP_MODULE_LSC;
	params.module_ens |= 0;
	params.module_cfg_update |= RKISP1_CIF_ISP_MODULE_LSC;
}

static void configureIe(struct rkisp1_params_cfg &params)
{
	params.module_en_update |= RKISP1_CIF_ISP_MODULE_IE;
	params.module_ens |= 0;
	params.module_cfg_update |= RKISP1_CIF_ISP_MODULE_IE;

	params.others.ie_config.effect = 0;
}

static void configureBdm(struct rkisp1_params_cfg &params)
{
	params.module_en_update |= RKISP1_CIF_ISP_MODULE_BDM;
	params.module_ens |= 0;
	params.module_cfg_update |= RKISP1_CIF_ISP_MODULE_BDM;

	params.others.bdm_config.demosaic_th = 4;
}

static void configureAec(struct rkisp1_params_cfg &params)
{
	params.module_en_update |= RKISP1_CIF_ISP_MODULE_AEC;
	params.module_ens |= RKISP1_CIF_ISP_MODULE_AEC;
	params.module_cfg_update = RKISP1_CIF_ISP_MODULE_AEC;

	params.meas.aec_config.meas_window.h_offs = (1232 / 5) / 4;
	params.meas.aec_config.meas_window.h_size = (1640 / 5) / 2;
	params.meas.aec_config.meas_window.v_offs = (1232 / 5) / 4;
	params.meas.aec_config.meas_window.v_size = (1232 / 5) / 2;
	params.meas.aec_config.autostop = RKISP1_CIF_ISP_EXP_CTRL_AUTOSTOP_0;
	params.meas.aec_config.mode = RKISP1_CIF_ISP_EXP_MEASURING_MODE_0;
}

static void configureHist(struct rkisp1_params_cfg &params)
{
	params.module_cfg_update |= RKISP1_CIF_ISP_MODULE_HST;
	params.module_en_update |= RKISP1_CIF_ISP_MODULE_HST;
	params.module_ens |= RKISP1_CIF_ISP_MODULE_HST;
	params.meas.hst_config.mode = RKISP1_CIF_ISP_HISTOGRAM_MODE_R_HISTOGRAM;
	params.meas.hst_config.meas_window.h_offs = (1640 / 4) / 5;
	params.meas.hst_config.meas_window.h_size = (1640 / 2) / 5;
	params.meas.hst_config.meas_window.v_offs = (1232 / 4) / 5;
	params.meas.hst_config.meas_window.v_size = (1232 / 2) / 5;
	for (int i = 0; i < RKISP1_CIF_ISP_HISTOGRAM_WEIGHT_GRIDS_SIZE; i++)
		params.meas.hst_config.hist_weight[i] = 1;
}

static void configureBls(struct rkisp1_params_cfg &params)
{
	params.module_en_update |= RKISP1_CIF_ISP_MODULE_BLS;
	params.module_ens |= RKISP1_CIF_ISP_MODULE_BLS;
	params.module_cfg_update |= RKISP1_CIF_ISP_MODULE_BLS;

	params.others.bls_config.enable_auto = 0;
	params.others.bls_config.en_windows = 0;
	params.others.bls_config.fixed_val.r = 160;
	params.others.bls_config.fixed_val.gr = 160;
	params.others.bls_config.fixed_val.gb = 160;
	params.others.bls_config.fixed_val.b = 160;
}

static void configureCproc(struct rkisp1_params_cfg &params)
{
	params.module_en_update |= RKISP1_CIF_ISP_MODULE_CPROC;
	params.module_ens |= RKISP1_CIF_ISP_MODULE_CPROC;
	params.module_cfg_update |= RKISP1_CIF_ISP_MODULE_CPROC;

	params.others.cproc_config.c_out_range = 1;
	params.others.cproc_config.y_in_range = 1;
	params.others.cproc_config.y_out_range = 0;
	params.others.cproc_config.contrast = 200;
	params.others.cproc_config.brightness = 0;
	params.others.cproc_config.sat = 0x80;
	params.others.cproc_config.hue = 0;
}

static void configureDpcc(struct rkisp1_params_cfg &params)
{
	params.module_en_update |= RKISP1_CIF_ISP_MODULE_DPCC;
	params.module_ens |= 0;
	params.module_cfg_update |= RKISP1_CIF_ISP_MODULE_DPCC;
}

static void configureFlt(struct rkisp1_params_cfg &params)
{
	params.module_en_update |= RKISP1_CIF_ISP_MODULE_FLT;
	params.module_ens |= 0;
	params.module_cfg_update |= RKISP1_CIF_ISP_MODULE_FLT;
}

static void configureDpf(struct rkisp1_params_cfg &params)
{
	params.module_en_update |= RKISP1_CIF_ISP_MODULE_DPF;
	params.module_ens |= 0;
	params.module_cfg_update |= RKISP1_CIF_ISP_MODULE_DPF;
}

static void configureDpfStrength(struct rkisp1_params_cfg &params)
{
	params.module_en_update |= RKISP1_CIF_ISP_MODULE_DPF_STRENGTH;
	params.module_ens |= 0;
	params.module_cfg_update |= RKISP1_CIF_ISP_MODULE_DPF_STRENGTH;
}

static void configureGoc(struct rkisp1_params_cfg &params)
{
	params.module_en_update |= RKISP1_CIF_ISP_MODULE_GOC;
	params.module_ens |= 0;
	params.module_cfg_update |= RKISP1_CIF_ISP_MODULE_GOC;
}

void IPARkISP1::configureParams()
{
	params_.module_en_update = 0;
	params_.module_ens = 0;
	params_.module_cfg_update = 0;

	configureAwb(params_);
	configureAwbGains(params_);
	configureCtk(params_);
	configureLsc(params_);

	configureAec(params_);
	configureHist(params_);
	configureBls(params_);
	configureDpcc(params_);
	configureFlt(params_);
	configureDpf(params_);
	configureDpfStrength(params_);

	configureCproc(params_);
	configureGoc(params_);
	configureIe(params_);
	configureBdm(params_);
}

/**
 * \todo The RkISP1 pipeline currently provides an empty IPACameraSensorInfo
 * if the connected sensor does not provide enough information to properly
 * assemble one. Make sure the reported sensor information are relevant
 * before accessing them.
 */
int IPARkISP1::configure([[maybe_unused]] const IPACameraSensorInfo &info,
			 [[maybe_unused]] const std::map<uint32_t, IPAStream> &streamConfig,
			 const std::map<uint32_t, ControlInfoMap> &entityControls)
{
	if (entityControls.empty())
		return -EINVAL;

	ctrls_ = entityControls.at(0);

	const auto itExp = ctrls_.find(V4L2_CID_EXPOSURE);
	if (itExp == ctrls_.end()) {
		LOG(IPARkISP1, Error) << "Can't find exposure control";
		return -EINVAL;
	}

	const auto itGain = ctrls_.find(V4L2_CID_ANALOGUE_GAIN);
	if (itGain == ctrls_.end()) {
		LOG(IPARkISP1, Error) << "Can't find gain control";
		return -EINVAL;
	}

	autoExposure_ = true;

	minExposure_ = std::max<uint32_t>(itExp->second.min().get<int32_t>(), 1);
	maxExposure_ = itExp->second.max().get<int32_t>();
	exposure_ = minExposure_;

	minGain_ = std::max<uint32_t>(itGain->second.min().get<int32_t>(), 1);
	maxGain_ = itGain->second.max().get<int32_t>();
	gain_ = minGain_;

	LOG(IPARkISP1, Info)
		<< "Exposure: " << minExposure_ << "-" << maxExposure_
		<< " Gain: " << minGain_ << "-" << maxGain_;

	params_ = {};
	configureParams();

	aeLocked_ = false;
	awbLocked_ = false;

	awbAlgo_ = std::make_unique<RkISP1Awb>();
	awbAlgo_->initialise(params_);

	return 0;
}

void IPARkISP1::mapBuffers(const std::vector<IPABuffer> &buffers)
{
	for (const IPABuffer &buffer : buffers) {
		auto elem = buffers_.emplace(std::piecewise_construct,
					     std::forward_as_tuple(buffer.id),
					     std::forward_as_tuple(buffer.planes));
		const FrameBuffer &fb = elem.first->second;

		/*
		 * \todo Provide a helper to mmap() buffers (possibly exposed
		 * to applications).
		 */
		buffersMemory_[buffer.id] = mmap(NULL,
						 fb.planes()[0].length,
						 PROT_READ | PROT_WRITE,
						 MAP_SHARED,
						 fb.planes()[0].fd.fd(),
						 0);

		if (buffersMemory_[buffer.id] == MAP_FAILED) {
			int ret = -errno;
			LOG(IPARkISP1, Fatal) << "Failed to mmap buffer: "
					      << strerror(-ret);
		}
	}
}

void IPARkISP1::unmapBuffers(const std::vector<unsigned int> &ids)
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

void IPARkISP1::processEvent(const RkISP1Event &event)
{
	switch (event.op) {
	case EventSignalStatBuffer: {
		unsigned int frame = event.frame;
		unsigned int bufferId = event.bufferId;

		const rkisp1_stat_buffer *stats =
			static_cast<rkisp1_stat_buffer *>(buffersMemory_[bufferId]);

		updateStatistics(frame, stats);
		break;
	}
	case EventQueueRequest: {
		unsigned int frame = event.frame;
		unsigned int bufferId = event.bufferId;

		rkisp1_params_cfg *params =
			static_cast<rkisp1_params_cfg *>(buffersMemory_[bufferId]);

		queueRequest(frame, params, event.controls);
		break;
	}
	default:
		LOG(IPARkISP1, Error) << "Unknown event " << event.op;
		break;
	}
}

void IPARkISP1::queueRequest(unsigned int frame, rkisp1_params_cfg *params,
			     const ControlList &controls)
{
	configureParams();

	/* Auto Exposure on/off. */
	if (controls.contains(controls::AeEnable)) {
		autoExposure_ = controls.get(controls::AeEnable);
		if (autoExposure_)
			params_.module_ens |= RKISP1_CIF_ISP_MODULE_AEC;

		params_.module_en_update |= RKISP1_CIF_ISP_MODULE_AEC;
	}

	if (awbLocked_ == false && aeLocked_ == true && (frame % 30 == 0)) {
		awbAlgo_->updateWbParameters(params_);
		awbLocked_ = true;
	}

	*params = params_;

	RkISP1Action op;
	op.op = ActionParamFilled;

	queueFrameAction.emit(frame, op);
}

void IPARkISP1::updateStatistics(unsigned int frame,
				 const rkisp1_stat_buffer *stats)
{
	const rkisp1_cif_isp_stat *params = &stats->params;
	unsigned int aeState = 0;
#if 0
	const rkisp1_cif_isp_hist_stat *hist = &params->hist;
	for (int i = 0 ; i < RKISP1_CIF_ISP_HIST_BIN_N_MAX ; i++) {
		LOG(IPARkISP1, Error) << i << ": " << (__u32)hist->hist_bins[i];
	}
#endif
	if (stats->meas_type & RKISP1_CIF_ISP_STAT_AUTOEXP) {
		const rkisp1_cif_isp_ae_stat *ae = &params->ae;

		const unsigned int target = 60;

		unsigned int value = 0;
		unsigned int num = 0;
		for (int i = 0; i < RKISP1_CIF_ISP_AE_MEAN_MAX_V10; i++) {
			if (ae->exp_mean[i] <= 15)
				continue;

			value += ae->exp_mean[i];
			num++;
		}
		value /= num;

		double factor = (double)target / value;

		if (frame % 3 == 0) {
			double exposure;

			exposure = factor * exposure_ * gain_ / minGain_;
			exposure_ = std::clamp<uint64_t>((uint64_t)exposure,
							 minExposure_,
							 maxExposure_);

			exposure = exposure / exposure_ * minGain_;
			gain_ = std::clamp<uint64_t>((uint64_t)exposure,
						     minGain_ + 1, maxGain_);

			setControls(frame + 1);
		}

		aeState = fabs(factor - 1.0f) < 0.05f ? 2 : 1;
	}

	if (aeState) {
		awbAlgo_->calculateWBGains(stats);
	}

	metadataReady(frame, aeState);
}

void IPARkISP1::setControls(unsigned int frame)
{
	RkISP1Action op;
	op.op = ActionV4L2Set;

	ControlList ctrls(ctrls_);
	ctrls.set(V4L2_CID_EXPOSURE, static_cast<int32_t>(exposure_));
	ctrls.set(V4L2_CID_ANALOGUE_GAIN, static_cast<int32_t>(gain_));
	op.controls = ctrls;

	queueFrameAction.emit(frame, op);
}

void IPARkISP1::metadataReady(unsigned int frame, unsigned int aeState)
{
	ControlList ctrls(controls::controls);

	if (aeState) {
		aeLocked_ = true;
		ctrls.set(controls::AeLocked, aeState == 2);
	}

	RkISP1Action op;
	op.op = ActionMetadata;
	op.controls = ctrls;

	queueFrameAction.emit(frame, op);
}

} /* namespace ipa::rkisp1 */

/*
 * External IPA module interface
 */

extern "C" {
const struct IPAModuleInfo ipaModuleInfo = {
	IPA_MODULE_API_VERSION,
	1,
	"PipelineHandlerRkISP1",
	"rkisp1",
};

IPAInterface *ipaCreate()
{
	return new ipa::rkisp1::IPARkISP1();
}
}

} /* namespace libcamera */
