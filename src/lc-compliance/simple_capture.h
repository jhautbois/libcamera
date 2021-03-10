/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * simple_capture.h - Simple capture helper
 */
#ifndef __LC_COMPLIANCE_SIMPLE_CAPTURE_H__
#define __LC_COMPLIANCE_SIMPLE_CAPTURE_H__

#include <memory>

#include <libcamera/libcamera.h>

#include "../cam/event_loop.h"
#include "results.h"

class SimpleCapture
{
public:
	Results::Result configure(libcamera::StreamRole role);

protected:
	SimpleCapture(std::shared_ptr<libcamera::Camera> camera);
	virtual ~SimpleCapture();

	Results::Result start();
	Results::Result stop();

	virtual void requestComplete(libcamera::Request *request) = 0;

	EventLoop *loop_;

	std::shared_ptr<libcamera::Camera> camera_;
	std::unique_ptr<libcamera::FrameBufferAllocator> allocator_;
	std::unique_ptr<libcamera::CameraConfiguration> config_;
};

class SimpleCaptureBalanced : public SimpleCapture
{
public:
	SimpleCaptureBalanced(std::shared_ptr<libcamera::Camera> camera);

	Results::Result capture(unsigned int numRequests);

private:
	int queueRequest(libcamera::Request *request);
	void requestComplete(libcamera::Request *request) override;

	unsigned int queueCount_;
	unsigned int captureCount_;
	unsigned int captureLimit_;
};

class SimpleCaptureUnbalanced : public SimpleCapture
{
public:
	SimpleCaptureUnbalanced(std::shared_ptr<libcamera::Camera> camera);

	Results::Result capture(unsigned int numRequests);

private:
	void requestComplete(libcamera::Request *request) override;

	unsigned int captureCount_;
	unsigned int captureLimit_;
};

#endif /* __LC_COMPLIANCE_SIMPLE_CAPTURE_H__ */
