/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * ipu3.cpp - IPU3 Image Processing Algorithms
 */

#include <stdint.h>
#include <sys/mman.h>

#include <linux/intel-ipu3.h>
#include <linux/v4l2-controls.h>

#include <libcamera/buffer.h>
#include <libcamera/control_ids.h>
#include <libcamera/ipa/ipa_interface.h>
#include <libcamera/ipa/ipa_module_info.h>
#include <libcamera/ipa/ipu3_ipa_interface.h>
#include <libcamera/request.h>

#include "libcamera/internal/buffer.h"
#include "libcamera/internal/log.h"

#include "ipu3_agc.h"
#include "ipu3_awb.h"

static const uint32_t kMaxCellWidthPerSet = 160;
static const uint32_t kMaxCellHeightPerSet = 80;

namespace libcamera {

LOG_DEFINE_CATEGORY(IPAIPU3)

class IPAIPU3 : public ipa::ipu3::IPAIPU3Interface
{
public:
	int init([[maybe_unused]] const IPASettings &settings) override
	{
		return 0;
	}
	int start() override;
	void stop() override {}

	void configure(const std::map<uint32_t, ControlInfoMap> &entityControls,
		       const Size &bdsOutputSize) override;

	void mapBuffers(const std::vector<IPABuffer> &buffers) override;
	void unmapBuffers(const std::vector<unsigned int> &ids) override;
	void processEvent(const ipa::ipu3::IPU3Event &event) override;

private:
	void processControls(unsigned int frame, const ControlList &controls);
	void fillParams(unsigned int frame, ipu3_uapi_params *params);
	void parseStatistics(unsigned int frame,
			     const ipu3_uapi_stats_3a *stats);

	void setControls(unsigned int frame);
	void calculateBdsGrid(const Size &bdsOutputSize);

	std::map<unsigned int, MappedFrameBuffer> buffers_;

	ControlInfoMap ctrls_;

	/* Camera sensor controls. */
	uint32_t exposure_;
	uint32_t minExposure_;
	uint32_t maxExposure_;
	uint32_t gain_;
	uint32_t minGain_;
	uint32_t maxGain_;

	/* Interface to the AWB algorithm */
	std::unique_ptr<ipa::IPU3Awb> awbAlgo_;
	/* Interface to the AEC/AGC algorithm */
	std::unique_ptr<ipa::IPU3Agc> agcAlgo_;
	/* Local parameter storage */
	struct ipu3_uapi_params params_;

	struct ipu3_uapi_grid_config bdsGrid_;
};

int IPAIPU3::start()
{
	setControls(0);

	return 0;
}

/* This method calculates a grid for the AWB algorithm in the IPU3 firmware.
 * Its input it the BDS output size calculated in the imgU.
 * It is limited for now to the simplest method: find the lesser error
 * with the width/height and respective log2 width/height of the cells.
 * \todo smaller cells are better so adapt x_start to lose a bit but have
 * a better average resolution. If a cell is saturated, it might make a big difference. */
void IPAIPU3::calculateBdsGrid(const Size &bdsOutputSize)
{
	std::vector<uint32_t> log2WidthVector = { 3, 4, 5, 6, 7 };
	std::vector<uint32_t> log2HeightVector = { 3, 4, 5, 6, 7 };
	uint32_t minError = std::numeric_limits<uint32_t>::max();
	uint32_t bestWidth = 0;
	uint32_t bestHeight = 0;
	uint32_t bestLog2Width = 0;
	uint32_t bestLog2Height = 0;
	bdsGrid_ = {};

	for (uint32_t i = 0; i < log2WidthVector.size(); ++i) {
		uint32_t width = std::min(kMaxCellWidthPerSet, bdsOutputSize.width >> log2WidthVector[i]);
		width = width << log2WidthVector[i];
		for (uint32_t j = 0; j < log2HeightVector.size(); ++j) {
			int32_t height = std::min(kMaxCellHeightPerSet, bdsOutputSize.height >> log2HeightVector[j]);
			height = height << log2HeightVector[j];
			uint32_t error = std::abs((int)(width - bdsOutputSize.width)) + std::abs((int)(height - bdsOutputSize.height));

			if (error > minError)
				continue;

			minError = error;
			bestWidth = width;
			bestHeight = height;
			bestLog2Width = log2WidthVector[i];
			bestLog2Height = log2HeightVector[j];
		}
	}

	bdsGrid_.width = bestWidth >> bestLog2Width;
	bdsGrid_.block_width_log2 = bestLog2Width;
	bdsGrid_.height = bestHeight >> bestLog2Height;
	bdsGrid_.block_height_log2 = bestLog2Height;
	LOG(IPAIPU3, Debug) << "Best grid found is: ("
			    << (int)bdsGrid_.width << " << " << (int)bdsGrid_.block_width_log2 << ") x ("
			    << (int)bdsGrid_.height << "<<" << (int)bdsGrid_.block_height_log2 << ")";
}

void IPAIPU3::configure(const std::map<uint32_t, ControlInfoMap> &entityControls,
			[[maybe_unused]] const Size &bdsOutputSize)
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

	params_ = {};

	calculateBdsGrid(bdsOutputSize);

	awbAlgo_ = std::make_unique<ipa::IPU3Awb>();
	awbAlgo_->initialise(params_, bdsOutputSize, bdsGrid_);

	agcAlgo_ = std::make_unique<ipa::IPU3Agc>();
	agcAlgo_->initialise(bdsGrid_);
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

void IPAIPU3::processEvent(const ipa::ipu3::IPU3Event &event)
{
	switch (event.op) {
	case ipa::ipu3::EventProcessControls: {
		processControls(event.frame, event.controls);
		break;
	}
	case ipa::ipu3::EventStatReady: {
		auto it = buffers_.find(event.bufferId);
		if (it == buffers_.end()) {
			LOG(IPAIPU3, Error) << "Could not find stats buffer!";
			return;
		}

		Span<uint8_t> mem = it->second.maps()[0];
		const ipu3_uapi_stats_3a *stats =
			reinterpret_cast<ipu3_uapi_stats_3a *>(mem.data());

		parseStatistics(event.frame, stats);
		break;
	}
	case ipa::ipu3::EventFillParams: {
		auto it = buffers_.find(event.bufferId);
		if (it == buffers_.end()) {
			LOG(IPAIPU3, Error) << "Could not find param buffer!";
			return;
		}

		Span<uint8_t> mem = it->second.maps()[0];
		ipu3_uapi_params *params =
			reinterpret_cast<ipu3_uapi_params *>(mem.data());

		fillParams(event.frame, params);
		break;
	}
	default:
		LOG(IPAIPU3, Error) << "Unknown event " << event.op;
		break;
	}
}

void IPAIPU3::processControls([[maybe_unused]] unsigned int frame,
			      [[maybe_unused]] const ControlList &controls)
{
	/* \todo Start processing for 'frame' based on 'controls'. */
}

void IPAIPU3::fillParams(unsigned int frame, ipu3_uapi_params *params)
{
	if (agcAlgo_->updateControls())
		awbAlgo_->updateWbParameters(params_, agcAlgo_->gamma());

	*params = params_;

	ipa::ipu3::IPU3Action op;
	op.op = ipa::ipu3::ActionParamFilled;

	queueFrameAction.emit(frame, op);
}

void IPAIPU3::parseStatistics(unsigned int frame,
			      [[maybe_unused]] const ipu3_uapi_stats_3a *stats)
{
	ControlList ctrls(controls::controls);

	if (!stats->stats_3a_status.awb_en) {
		LOG(IPAIPU3, Error) << "AWB stats are not enabled";
	} else {
		agcAlgo_->process(stats, exposure_, gain_);
		awbAlgo_->calculateWBGains(stats);
		if (agcAlgo_->updateControls())
			setControls(frame);
	}

	ipa::ipu3::IPU3Action op;
	op.op = ipa::ipu3::ActionMetadataReady;
	op.controls = ctrls;

	queueFrameAction.emit(frame, op);
}

void IPAIPU3::setControls(unsigned int frame)
{
	ipa::ipu3::IPU3Action op;
	op.op = ipa::ipu3::ActionSetSensorControls;

	ControlList ctrls(ctrls_);
	ctrls.set(V4L2_CID_EXPOSURE, static_cast<int32_t>(exposure_));
	ctrls.set(V4L2_CID_ANALOGUE_GAIN, static_cast<int32_t>(gain_));
	op.controls = ctrls;

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

IPAInterface *ipaCreate()
{
	return new IPAIPU3();
}
}

} /* namespace libcamera */
