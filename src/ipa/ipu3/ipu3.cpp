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

#include "ipu3_awb.h"

static constexpr uint32_t kMaxCellWidthPerSet = 160;
static constexpr uint32_t kMaxCellHeightPerSet = 80;

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

	/* Local parameter storage */
	struct ipu3_uapi_params params_;

	struct ipu3_uapi_grid_config bdsGrid_;
};

int IPAIPU3::start()
{
	setControls(0);

	return 0;
}

/**
 * This method calculates a grid for the AWB algorithm in the IPU3 firmware.
 * Its input is the BDS output size calculated in the ImgU.
 * It is limited for now to the simplest method: find the lesser error
 * with the width/height and respective log2 width/height of the cells.
 *
 * \todo The frame is divided into cells which can be 8x8 => 128x128.
 * As a smaller cell improves the algorithm precision, adapting the
 * x_start and y_start parameters of the grid would provoke a loss of
 * some pixels but would also result in more accurate algorithms.
 */
void IPAIPU3::calculateBdsGrid(const Size &bdsOutputSize)
{
	uint32_t minError = std::numeric_limits<uint32_t>::max();
	Size best;
	Size bestLog2;
	bdsGrid_ = {};

	for (uint32_t widthShift = 3; widthShift <= 7; ++widthShift) {
		uint32_t width = std::min(kMaxCellWidthPerSet,
					  bdsOutputSize.width >> widthShift);
		width = width << widthShift;
		for (uint32_t heightShift = 3; heightShift <= 7; ++heightShift) {
			int32_t height = std::min(kMaxCellHeightPerSet,
						  bdsOutputSize.height >> heightShift);
			height = height << heightShift;
			uint32_t error  = std::abs(static_cast<int>(width - bdsOutputSize.width))
							+ std::abs(static_cast<int>(height - bdsOutputSize.height));

			if (error > minError)
				continue;

			minError = error;
			best.width = width;
			best.height = height;
			bestLog2.width = widthShift;
			bestLog2.height = heightShift;
		}
	}

	bdsGrid_.width = best.width >> bestLog2.width;
	bdsGrid_.block_width_log2 = bestLog2.width;
	bdsGrid_.height = best.height >> bestLog2.height;
	bdsGrid_.block_height_log2 = bestLog2.height;

	LOG(IPAIPU3, Debug) << "Best grid found is: ("
			    << (int)bdsGrid_.width << " << " << (int)bdsGrid_.block_width_log2 << ") x ("
			    << (int)bdsGrid_.height << " <<" << (int)bdsGrid_.block_height_log2 << ")";
}

void IPAIPU3::configure(const std::map<uint32_t, ControlInfoMap> &entityControls,
			const Size &bdsOutputSize)
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
	awbAlgo_->updateWbParameters(params_, 1.0);

	*params = params_;

	ipa::ipu3::IPU3Action op;
	op.op = ipa::ipu3::ActionParamFilled;

	queueFrameAction.emit(frame, op);
}

void IPAIPU3::parseStatistics(unsigned int frame,
			      [[maybe_unused]] const ipu3_uapi_stats_3a *stats)
{
	ControlList ctrls(controls::controls);

	awbAlgo_->calculateWBGains(stats);

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
