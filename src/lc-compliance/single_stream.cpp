/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * single_stream.cpp - Test a single camera stream
 */

#include <iostream>

#include "../cam/event_loop.h"
#include "tests.h"

using namespace libcamera;

class SimpleCapture
{
public:
	SimpleCapture(std::shared_ptr<Camera> camera)
		: camera_(camera), allocator_(std::make_unique<FrameBufferAllocator>(camera))
	{
	}

	Results::Result configure(StreamRole role);
	Results::Result start();
	Results::Result capture(unsigned int numRequests);
	Results::Result stop();

private:
	int queueRequest(Request *request);
	void requestComplete(Request *request);

	std::shared_ptr<libcamera::Camera> camera_;
	std::unique_ptr<FrameBufferAllocator> allocator_;
	std::unique_ptr<libcamera::CameraConfiguration> config_;

	EventLoop *loop_;
	unsigned int queueCount_;
	unsigned int captureCount_;
	unsigned int captureLimit_;
};

Results::Result SimpleCapture::configure(StreamRole role)
{
	config_ = camera_->generateConfiguration({ role });

	if (config_->validate() != CameraConfiguration::Valid) {
		config_.reset();
		return { Results::Fail, "Configuration not valid" };
	}

	if (camera_->configure(config_.get())) {
		config_.reset();
		return { Results::Fail, "Failed to configure camera" };
	}

	return { Results::Pass, "Configure camera" };
}

Results::Result SimpleCapture::start()
{
	Stream *stream = config_->at(0).stream();
	if (allocator_->allocate(stream) < 0)
		return { Results::Fail, "Failed to allocate buffers" };

	if (camera_->start())
		return { Results::Fail, "Failed to start camera" };

	camera_->requestCompleted.connect(this, &SimpleCapture::requestComplete);

	return { Results::Pass, "Started camera" };
}

Results::Result SimpleCapture::capture(unsigned int numRequests)
{
	Stream *stream = config_->at(0).stream();
	const std::vector<std::unique_ptr<FrameBuffer>> &buffers = allocator_->buffers(stream);

	/* No point in testing less requests then the camera depth. */
	if (buffers.size() > numRequests)
		return { Results::Skip, "Camera needs " + std::to_string(buffers.size()) + " requests, can't test only " + std::to_string(numRequests) };

	queueCount_ = 0;
	captureCount_ = 0;
	captureLimit_ = numRequests;

	/* Queue the recommended number of reqeuests. */
	std::vector<std::unique_ptr<libcamera::Request>> requests;
	for (const std::unique_ptr<FrameBuffer> &buffer : allocator_->buffers(stream)) {
		std::unique_ptr<Request> request = camera_->createRequest();
		if (!request)
			return { Results::Fail, "Can't create request" };

		if (request->addBuffer(stream, buffer.get()))
			return { Results::Fail, "Can't set buffer for request" };

		if (queueRequest(request.get()) < 0)
			return { Results::Fail, "Failed to queue request" };

		requests.push_back(std::move(request));
	}

	/* Run capture session. */
	loop_ = new EventLoop();
	loop_->exec();
	delete loop_;

	if (captureCount_ != captureLimit_)
		return { Results::Fail, "Got " + std::to_string(captureCount_) + " request, wanted " + std::to_string(captureLimit_) };

	return { Results::Pass, "Balanced capture of " + std::to_string(numRequests) + " requests" };
}

Results::Result SimpleCapture::stop()
{
	Stream *stream = config_->at(0).stream();

	camera_->stop();

	camera_->requestCompleted.disconnect(this, &SimpleCapture::requestComplete);

	allocator_->free(stream);

	return { Results::Pass, "Stopped camera" };
}

int SimpleCapture::queueRequest(Request *request)
{
	queueCount_++;
	if (queueCount_ > captureLimit_)
		return 0;

	return camera_->queueRequest(request);
}

void SimpleCapture::requestComplete(Request *request)
{
	captureCount_++;
	if (captureCount_ >= captureLimit_) {
		loop_->exit(0);
		return;
	}

	request->reuse(Request::ReuseBuffers);
	if (queueRequest(request))
		loop_->exit(-EINVAL);
}

Results::Result testRequestBalance(std::shared_ptr<Camera> camera,
				   unsigned int startCycles,
				   unsigned int numRequests)
{
	SimpleCapture capture(camera);
	Results::Result ret;

	ret = capture.configure(Viewfinder);
	if (ret.first != Results::Pass)
		return ret;

	for (unsigned int starts = 0; starts < startCycles; starts++) {
		ret = capture.start();
		if (ret.first != Results::Pass)
			return ret;

		ret = capture.capture(numRequests);
		if (ret.first != Results::Pass) {
			capture.stop();
			return ret;
		}

		ret = capture.stop();
		if (ret.first != Results::Pass)
			return ret;
	}

	return { Results::Pass, "Balanced capture of " + std::to_string(numRequests) + " requests with " + std::to_string(startCycles) + " start cycles" };
}

Results testSingleStream(std::shared_ptr<Camera> camera)
{
	const std::vector<unsigned int> numRequests = { 1, 2, 3, 5, 8, 13, 21, 34, 55, 89 };

	Results results(numRequests.size() * 2);

	if (!camera)
		return results;

	/*
	 * Test single capture cycles
	 *
	 * Makes sure the camera completes the exact number of requests queued.
	 * Example failure is a camera that needs N+M requests queued to
	 * complete N requests to the application.
	 */
	std::cout << "Test single capture cycles" << std::endl;
	for (unsigned int num : numRequests)
		results.add(testRequestBalance(camera, 1, num));

	/*
	 * Test multiple start/stop cycles
	 *
	 * Makes sure the camera supports multiple start/stop cycles.
	 * Example failure is a camera that does not clean up correctly in its
	 * error path but is only tested by single-capture applications.
	 */
	std::cout << "Test multiple start/stop cycles" << std::endl;
	for (unsigned int num : numRequests)
		results.add(testRequestBalance(camera, 3, num));

	return results;
}
