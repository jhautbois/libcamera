/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Red Hat
 * Copyright (C) 2022, Ideas On Board
 *
 * af.cpp - automatic contrast-based focus algorithm
 */
#include <cmath>

#include <stdint.h>

#include <libcamera/base/log.h>

#include "af.h"

using namespace RPiController;
using namespace libcamera;

LOG_DEFINE_CATEGORY(IoBAf)

#define NAME "iob.af"

/*
 * Maximum focus steps of the VCM control
 * \todo should be obtained from the VCM driver
 */
static constexpr uint32_t kMaxFocusSteps = 1023;

/* Minimum focus step for searching appropriate focus */
static constexpr uint32_t kCoarseSearchStep = 30;
static constexpr uint32_t kFineSearchStep = 1;

/* Max ratio of variance change, 0.0 < kMaxChange < 1.0 */
static constexpr double kMaxChange = 0.5;

/* Fine scan range 0 < kFineRange < 1 */
static constexpr double kFineRange = 0.05;

Af::Af(Controller *controller)
	: AfAlgorithm(controller), focus_(0), bestFocus_(0),
	  currentContrast_(0.0), previousContrast_(0.0), maxContrast_(0.0),
	  maxStep_(0), coarseCompleted_(false), fineCompleted_(false),
	  mode_(0), lowStep_(0), highStep_(kMaxFocusSteps)
{
}

char const *Af::Name() const
{
	return NAME;
}

void Af::SetMode([[maybe_unused]] const uint32_t &mode)
{
	mode_ = mode;
}

void Af::Trigger()
{
}

void Af::Cancel()
{
}

void Af::SetWindows([[maybe_unused]] const libcamera::Rectangle &afWindows)
{
}

void Af::SetRange(const uint32_t &low, const uint32_t &high)
{
	lowStep_ = low;
	highStep_ = high;

	LOG(IoBAf, Debug) << "Lens range set between " << lowStep_
			  << " and " << highStep_;

	focus_ = lowStep_;
	maxStep_ = highStep_;
}

void Af::setSpeed([[maybe_unused]] const uint32_t &speed)
{
}

void Af::Initialise()
{
	status_.lensPosition = 0.0;
	maxContrast_ = 0.0;
	status_.state = 1;
}

void Af::Prepare(Metadata *image_metadata)
{
	image_metadata->Set("af.status", status_);
}

void Af::afCoarseScan()
{
	if (coarseCompleted_)
		return;

	if (afScan(kCoarseSearchStep)) {
		coarseCompleted_ = true;
		maxContrast_ = 0;
		focus_ = status_.lensPosition - (status_.lensPosition * kFineRange);
		status_.lensPosition = focus_;
		previousContrast_ = 0;
		maxStep_ = std::clamp(focus_ + static_cast<uint32_t>((focus_ * kFineRange)),
				      0U, highStep_);
	}
}

void Af::afFineScan()
{
	if (!coarseCompleted_)
		return;

	if (afScan(kFineSearchStep)) {
		status_.state = 2;
		fineCompleted_ = true;
	}
}

bool Af::afScan(uint32_t minSteps)
{
	if (focus_ + minSteps > maxStep_) {
		/* If the max step is reached, move lens to the position. */
		status_.lensPosition = bestFocus_;
		return true;
	} else {
		/*
		 * Find the maximum of the variance by estimating its
		 * derivative. If the direction changes, it means we have passed
		 * a maximum one step before.
		 */
		if ((currentContrast_ - maxContrast_) >= -(maxContrast_ * 0.1)) {
			/*
			 * Positive and zero derivative:
			 * The variance is still increasing. The focus could be
			 * increased for the next comparison. Also, the max
			 * variance and previous focus value are updated.
			 */
			bestFocus_ = focus_;
			focus_ += minSteps;
			maxContrast_ = currentContrast_;
			status_.lensPosition = focus_;
		} else {
			/*
			 * Negative derivative:
			 * The variance starts to decrease which means the maximum
			 * variance is found. Set focus step to previous good one
			 * then return immediately.
			 */
			status_.lensPosition = bestFocus_;
			return true;
		}
	}

	previousContrast_ = currentContrast_;
	LOG(IoBAf, Debug) << " Previous step is "
			  << bestFocus_
			  << " Current step is "
			  << focus_;
	return false;
}

void Af::afReset()
{
	status_.lensPosition = lowStep_;
	focus_ = lowStep_;
	maxStep_ = highStep_;
	status_.state = 0;
	previousContrast_ = 0.0;
	coarseCompleted_ = false;
	fineCompleted_ = false;
	maxContrast_ = 0.0;
}

bool Af::afIsOutOfFocus()
{
	const uint32_t diff_var = std::abs(currentContrast_ -
					   maxContrast_);
	const double var_ratio = diff_var / maxContrast_;
	LOG(IoBAf, Debug) << "Variance change rate: "
			  << var_ratio
			  << " Current VCM step: "
			  << status_.lensPosition;
	if (var_ratio > kMaxChange)
		return true;
	else
		return false;
}

void Af::Process(StatisticsPtr &stats, [[maybe_unused]] Metadata *image_metadata)
{
	unsigned int i;
	currentContrast_ = 0.0;

	/* Use the second filter results only, and cache those. */
	for (i = 0; i < FOCUS_REGIONS; i++)
		currentContrast_ += stats->focus_stats[i].contrast_val[1][1]
				  / stats->focus_stats[i].contrast_val_num[1][1];

	if (status_.state != 2) {
		afCoarseScan();
		afFineScan();
	} else {
		if (afIsOutOfFocus())
			afReset();
	}
}

/* Register algorithm with the system. */
static Algorithm *Create(Controller *controller)
{
	return new Af(controller);
}
static RegisterAlgorithm reg(NAME, &Create);
