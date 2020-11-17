/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * delayed_controls.h - Helper to deal with controls that are applied with a delay
 */

#include "libcamera/internal/delayed_controls.h"
#include "libcamera/internal/v4l2_device.h"

#include <libcamera/controls.h>

#include "libcamera/internal/log.h"

/**
 * \file delayed_controls.h
 * \brief Helper to deal with controls that are applied with a delay
 */

namespace libcamera {

LOG_DEFINE_CATEGORY(DelayedControls)

/**
 * \class DelayedControls
 * \brief Helper to deal with controls that take effect with a delay
 *
 * Some sensor controls take effect with a delay as the sensor needs time to
 * adjust, for example exposure and focus. This is an optional helper class to
 * deal with such controls and the intended users are pipeline handlers.
 *
 * The idea is to extend the concept of the buffer depth of a pipeline the
 * application need to maintain to also cover controls. Just as with buffer
 * depth if the application keeps the number of requests queued above the
 * control depth the controls are guaranteed to take effect for the correct
 * request. The control depth is determined by the control with the grates
 * delay.
 */

/**
 * \brief Construct a DelayedControls
 * \param[in] device The V4L2 device the controls have to be applied to
 * \param[in] delays Map of the numerical V4L2 control ids to their associated
 * delays (in frames)
 *
 * Only controls specified in \a delays are handled. If it's desired to mix
 * delayed controls and controls that take effect immediately the immediate
 * controls must be listed in the \a delays map with a delay value of 0.
 */
DelayedControls::DelayedControls(V4L2Device *device,
				 const std::unordered_map<uint32_t, unsigned int> &delays)
	: device_(device), maxDelay_(0)
{
	const ControlInfoMap &controls = device_->controls();

	/*
	 * Create a map of control ids to delays for controls exposed by the
	 * device.
	 */
	for (auto const &delay : delays) {
		auto itControl = controls.find(delay.first);
		if (itControl == controls.end()) {
			LOG(DelayedControls, Error)
				<< "Delay request for control id "
				<< utils::hex(delay.first)
				<< " but control is not exposed by device "
				<< device_->deviceNode();
			continue;
		}

		const ControlId *id = itControl->first;

		delays_[id] = delay.second;

		LOG(DelayedControls, Debug)
			<< "Set a delay of " << delays_[id]
			<< " for " << id->name();

		maxDelay_ = std::max(maxDelay_, delays_[id]);
	}

	reset();
}

/**
 * \brief Reset state machine and controls
 * \param[in] controls Optional list of controls to apply to the device
 *
 * Resets the state machine to a starting position based on control values
 * retrieved from the device. Controls may optionally be set before they are
 * read back using \a controls.
 */
void DelayedControls::reset(ControlList *controls)
{
	std::lock_guard<std::mutex> lock(lock_);

	running_ = false;
	fistSequence_ = 0;
	queueCount_ = 0;
	writeCount_ = 0;

	/* Set the controls on the device if requested. */
	if (controls)
		device_->setControls(controls);

	/* Retrieve current control values reported by the device. */
	std::vector<uint32_t> ids;
	for (auto const &delay : delays_)
		ids.push_back(delay.first->id());

	ControlList devCtrls = device_->getControls(ids);

	/* Seed the control queue with the controls reported by the device. */
	ctrls_.clear();
	for (const auto &ctrl : devCtrls) {
		const ControlId *id = devCtrls.infoMap()->idmap().at(ctrl.first);
		ctrls_[id][queueCount_] = ControlInfo(ctrl.second);
	}

	queueCount_++;
}

/**
 * \brief Push a set of controls on the queue
 * \param[in] controls List of controls to add to the device queue
 *
 * Push a set of controls to the control queue. This increases the control queue
 * depth by one.
 *
 * \returns true if \a controls are accepted, or false otherwise
 */
bool DelayedControls::push(const ControlList &controls)
{
	std::lock_guard<std::mutex> lock(lock_);

	return queue(controls);
}

bool DelayedControls::queue(const ControlList &controls)
{
	/* Copy state from previous frame. */
	for (auto &ctrl : ctrls_) {
		ControlInfo &info = ctrl.second[queueCount_];
		info.value = ctrls_[ctrl.first][queueCount_ - 1].value;
		info.updated = false;
	}

	/* Update with new controls. */
	for (const auto &control : controls) {
		const ControlIdMap &idmap = device_->controls().idmap();
		if (idmap.find(control.first) == idmap.end())
			return false;

		const ControlId *id = idmap.at(control.first);

		if (delays_.find(id) == delays_.end())
			return false;

		ControlInfo &info = ctrls_[id][queueCount_];

		info.value = control.second;
		info.updated = true;

		LOG(DelayedControls, Debug)
			<< "Queuing " << id->name()
			<< " to " << info.value.toString()
			<< " at index " << queueCount_;
	}

	queueCount_++;

	return true;
}

/**
 * \brief Read back controls in effect at a sequence number
 * \param[in] sequence The sequence number to get controls for
 *
 * Read back what controls where in effect at a specific sequence number. The
 * history is a ring buffer of 16 entries where new and old values coexist. It's
 * the callers responsibility to not read to old sequence numbers that have been
 * pushed out of the history.
 *
 * Historic values are evicted by pushing new values onto the queue using
 * push(). The max history from the current sequence number that yields valid
 * values are thus 16 minus number of controls pushed.
 *
 * \return The controls at \a sequence number
 */
ControlList DelayedControls::get(uint32_t sequence)
{
	std::lock_guard<std::mutex> lock(lock_);

	uint32_t adjustedSeq = sequence - fistSequence_ + 1;
	unsigned int index = std::max<int>(0, adjustedSeq - maxDelay_);

	ControlList out(device_->controls());
	for (const auto &ctrl : ctrls_) {
		const ControlId *id = ctrl.first;
		const ControlInfo &info = ctrl.second[index];

		out.set(id->id(), info.value);

		LOG(DelayedControls, Debug)
			<< "Reading " << id->name()
			<< " to " << info.value.toString()
			<< " at index " << index;
	}

	return out;
}

/**
 * \brief Inform DelayedControls of the start of a new frame
 * \param[in] sequence Sequence number of the frame that started
 *
 * Inform the state machine that a new frame has started and of its sequence
 * number. Any user of these helpers is responsible to inform the helper about
 * the start of any frame.This can be connected with ease to the start of a
 * exposure (SOE) V4L2 event.
 */
void DelayedControls::frameStart(uint32_t sequence)
{
	LOG(DelayedControls, Debug) << "frame " << sequence << " started";

	std::lock_guard<std::mutex> lock(lock_);

	if (!running_) {
		fistSequence_ = sequence;
		running_ = true;
	}

	/*
	 * Create control list peaking ahead in the value queue to ensure
	 * values are set in time to satisfy the sensor delay.
	 */
	ControlList out(device_->controls());
	for (const auto &ctrl : ctrls_) {
		const ControlId *id = ctrl.first;
		unsigned int delayDiff = maxDelay_ - delays_[id];
		unsigned int index = std::max<int>(0, writeCount_ - delayDiff);
		const ControlInfo &info = ctrl.second[index];

		if (info.updated) {
			out.set(id->id(), info.value);
			LOG(DelayedControls, Debug)
				<< "Setting " << id->name()
				<< " to " << info.value.toString()
				<< " at index " << index;
		}
	}

	writeCount_++;

	while (writeCount_ >= queueCount_) {
		LOG(DelayedControls, Debug)
			<< "Queue is empty, auto queue no-op.";
		queue({});
	}

	device_->setControls(&out);
}

} /* namespace libcamera */
