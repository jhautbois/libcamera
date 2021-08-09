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

#include <libcamera/base/log.h>

#include <libcamera/control_ids.h>
#include <libcamera/framebuffer.h>
#include <libcamera/ipa/ipa_interface.h>
#include <libcamera/ipa/ipa_module_info.h>
#include <libcamera/ipa/ipu3_ipa_interface.h>
#include <libcamera/request.h>

#include "libcamera/internal/framebuffer.h"

#include "algorithms/algorithm.h"
#include "algorithms/awb.h"
#include "algorithms/contrast.h"
#include "ipa_context.h"

#include "ipu3_agc.h"
#include "libipa/camera_sensor_helper.h"

static constexpr uint32_t kMaxCellWidthPerSet = 160;
static constexpr uint32_t kMaxCellHeightPerSet = 56;

namespace libcamera {

LOG_DEFINE_CATEGORY(IPAIPU3)

namespace ipa::ipu3 {

class IPAIPU3 : public IPAIPU3Interface
{
public:
	int init(const IPASettings &settings) override;
	int start() override;
	void stop() override {}

	int configure(const IPAConfigInfo &configInfo) override;

	void mapBuffers(const std::vector<IPABuffer> &buffers) override;
	void unmapBuffers(const std::vector<unsigned int> &ids) override;
	void processEvent(const IPU3Event &event) override;

private:
	void processControls(unsigned int frame, const ControlList &controls);
	void fillParams(unsigned int frame, ipu3_uapi_params *params);

	int initialiseAlgorithms();
	int configureAlgorithms();
	void processAlgorithms(const ipu3_uapi_stats_3a *stats);
	void parseStatistics(unsigned int frame,
			     int64_t frameTimestamp,
			     const ipu3_uapi_stats_3a *stats);

	void setControls(unsigned int frame);
	void calculateBdsGrid(const Size &bdsOutputSize);

	std::map<unsigned int, MappedFrameBuffer> buffers_;

	ControlInfoMap ctrls_;

	IPACameraSensorInfo sensorInfo_;

	/* Camera sensor controls. */
	uint32_t defVBlank_;
	uint32_t exposure_;
	uint32_t minExposure_;
	uint32_t maxExposure_;
	uint32_t gain_;
	uint32_t minGain_;
	uint32_t maxGain_;

	/* Interface to the AEC/AGC algorithm */
	std::unique_ptr<IPU3Agc> agcAlgo_;
	/* Interface to the Camera Helper */
	std::unique_ptr<CameraSensorHelper> camHelper_;

	/* Maintain the algorithms used by the IPA */
	std::list<std::unique_ptr<ipa::ipu3::Algorithm>> algorithms_;

	/* Local parameter storage */
	struct ipu3_uapi_grid_config bdsGrid_;
	struct IPAContext context_;
};

int IPAIPU3::init(const IPASettings &settings)
{
	camHelper_ = CameraSensorHelperFactory::create(settings.sensorModel);
	if (camHelper_ == nullptr) {
		LOG(IPAIPU3, Error) << "Failed to create camera sensor helper for " << settings.sensorModel;
		return -ENODEV;
	}

	/* Construct our Algorithms */
	algorithms_.emplace_back(new algorithms::Awb());
	algorithms_.emplace_back(new algorithms::Contrast());

	/* Reset all the hardware settings */
	context_.params = {};

	initialiseAlgorithms();

	return 0;
}

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
	ipu3_uapi_grid_config &bdsGrid = context_.awb.grid.bdsGrid;

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

	bdsGrid.width = best.width >> bestLog2.width;
	bdsGrid.block_width_log2 = bestLog2.width;
	bdsGrid.height = best.height >> bestLog2.height;
	bdsGrid.block_height_log2 = bestLog2.height;

	LOG(IPAIPU3, Debug) << "Best grid found is: ("
			    << (int)bdsGrid.width << " << " << (int)bdsGrid.block_width_log2 << ") x ("
			    << (int)bdsGrid.height << " << " << (int)bdsGrid.block_height_log2 << ")";
}

int IPAIPU3::configure(const IPAConfigInfo &configInfo)
{
	if (configInfo.entityControls.empty()) {
		LOG(IPAIPU3, Error) << "No controls provided";
		return -ENODATA;
	}

	sensorInfo_ = configInfo.sensorInfo;

	ctrls_ = configInfo.entityControls.at(0);

	const auto itExp = ctrls_.find(V4L2_CID_EXPOSURE);
	if (itExp == ctrls_.end()) {
		LOG(IPAIPU3, Error) << "Can't find exposure control";
		return -EINVAL;
	}

	const auto itGain = ctrls_.find(V4L2_CID_ANALOGUE_GAIN);
	if (itGain == ctrls_.end()) {
		LOG(IPAIPU3, Error) << "Can't find gain control";
		return -EINVAL;
	}

	const auto itVBlank = ctrls_.find(V4L2_CID_VBLANK);
	if (itVBlank == ctrls_.end()) {
		LOG(IPAIPU3, Error) << "Can't find VBLANK control";
		return -EINVAL;
	}

	minExposure_ = std::max(itExp->second.min().get<int32_t>(), 1);
	maxExposure_ = itExp->second.max().get<int32_t>();
	exposure_ = minExposure_;

	minGain_ = std::max(itGain->second.min().get<int32_t>(), 1);
	maxGain_ = itGain->second.max().get<int32_t>();
	gain_ = minGain_;

	defVBlank_ = itVBlank->second.def().get<int32_t>();

	/* Prepare AWB parameters */
	calculateBdsGrid(configInfo.bdsOutputSize);
	context_.awb.grid.bdsOutputSize = configInfo.bdsOutputSize;

	configureAlgorithms();

	bdsGrid_ = context_.awb.grid.bdsGrid;
	agcAlgo_ = std::make_unique<IPU3Agc>();
	agcAlgo_->initialise(bdsGrid_, sensorInfo_);

	return 0;
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

void IPAIPU3::processEvent(const IPU3Event &event)
{
	switch (event.op) {
	case EventProcessControls: {
		processControls(event.frame, event.controls);
		break;
	}
	case EventStatReady: {
		auto it = buffers_.find(event.bufferId);
		if (it == buffers_.end()) {
			LOG(IPAIPU3, Error) << "Could not find stats buffer!";
			return;
		}

		Span<uint8_t> mem = it->second.maps()[0];
		const ipu3_uapi_stats_3a *stats =
			reinterpret_cast<ipu3_uapi_stats_3a *>(mem.data());

		parseStatistics(event.frame, event.frameTimestamp, stats);
		break;
	}
	case EventFillParams: {
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
	*params = context_.params;

	IPU3Action op;
	op.op = ActionParamFilled;

	queueFrameAction.emit(frame, op);
}

int IPAIPU3::initialiseAlgorithms()
{
	for (auto const &algo : algorithms_) {
		int ret = algo->initialise(context_);
		if (ret)
			return ret;
	}

	return 0;
}

int IPAIPU3::configureAlgorithms()
{
	for (auto const &algo : algorithms_) {
		int ret = algo->configure(context_);
		if (ret)
			return ret;
	}

	return 0;
}

void IPAIPU3::processAlgorithms(const ipu3_uapi_stats_3a *stats)
{
	/* It might be better to pass the stats in as a parameter to process() ? */
	context_.stats = stats;

	for (auto const &algo : algorithms_) {
		algo->process(context_);
	}
}

void IPAIPU3::parseStatistics(unsigned int frame,
			      [[maybe_unused]] int64_t frameTimestamp,
			      [[maybe_unused]] const ipu3_uapi_stats_3a *stats)
{
	ControlList ctrls(controls::controls);

	/* Run the process for each algorithm on the stats */
	processAlgorithms(stats);

	double gain = camHelper_->gain(gain_);
	agcAlgo_->process(stats, exposure_, gain);
	gain_ = camHelper_->gainCode(gain);

	if (agcAlgo_->updateControls())
		setControls(frame);

	/* \todo Use VBlank value calculated from each frame exposure. */
	int64_t frameDuration = sensorInfo_.lineLength * (defVBlank_ + sensorInfo_.outputSize.height) /
				(sensorInfo_.pixelRate / 1e6);
	ctrls.set(controls::FrameDuration, frameDuration);

	IPU3Action op;
	op.op = ActionMetadataReady;
	op.controls = ctrls;

	queueFrameAction.emit(frame, op);
}

void IPAIPU3::setControls(unsigned int frame)
{
	IPU3Action op;
	op.op = ActionSetSensorControls;

	ControlList ctrls(ctrls_);
	ctrls.set(V4L2_CID_EXPOSURE, static_cast<int32_t>(exposure_));
	ctrls.set(V4L2_CID_ANALOGUE_GAIN, static_cast<int32_t>(gain_));
	op.controls = ctrls;

	queueFrameAction.emit(frame, op);
}

} /* namespace ipa::ipu3 */

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
	return new ipa::ipu3::IPAIPU3();
}
}

} /* namespace libcamera */
