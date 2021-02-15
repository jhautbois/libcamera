/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * ipa_algorithm.h - ISP control algorithm interface
 */
#ifndef __LIBCAMERA_IPA_ALGORITHM_H__
#define __LIBCAMERA_IPA_ALGORITHM_H__

/* All algorithms should be derived from this class and made available to the
 * Controller. */

#include <string>
#include <memory>
#include <map>

#include "ipa_controller.h"

namespace libcamera {

/* This defines the basic interface for all control algorithms. */

class IPAAlgorithm
{
public:
	IPAAlgorithm(IPAController *controller)
		: controller_(controller), paused_(false)
	{
	}
	virtual ~IPAAlgorithm() = default;
	virtual char const *Name() const = 0;
	virtual bool IsPaused() const { return paused_; }
	virtual void Pause() { paused_ = true; }
	virtual void Resume() { paused_ = false; }
	virtual void Initialise();
	virtual void Prepare();
	virtual void Process();

private:
	[[maybe_unused]] IPAController *controller_;
	bool paused_;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_IPA_ALGORITHM_H__ */
