/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Red Hat
 * Copyright (C) 2022, Ideas On Board
 *
 * focus.cpp - focus algorithm
 */
#include <cmath>

#include <stdint.h>

#include <libcamera/base/log.h>

#include "../focus_status.h"
#include "focus.hpp"

using namespace RPiController;
using namespace libcamera;

LOG_DEFINE_CATEGORY(IoBFocus)

#define NAME "iob.focus"

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

/* The numbers of frame to be ignored, before performing focus scan. */
static constexpr uint32_t kIgnoreFrame = 10;

/* Fine scan range 0 < kFineRange < 1 */
static constexpr double kFineRange = 0.05;

Focus::Focus(Controller *controller)
	: Algorithm(controller), focus_(0), bestFocus_(0), ignoreCounter_(0),
	  currentVariance_(0.0), previousVariance_(0.0), maxStep_(0),
	  coarseCompleted_(false), fineCompleted_(false)
{
}

char const *Focus::Name() const
{
	return NAME;
}

void Focus::Initialise()
{
	status_.focus = 0.0;
	status_.maxVariance = 0.0;
	status_.stable = false;
}

void Focus::Prepare(Metadata *image_metadata)
{
	image_metadata->Set("focus.status", status_);
}

double Focus::estimateVariance()
{
	unsigned int i;
	double mean;
	uint64_t total = 0;
	double var_sum = 0.0;

	/* Compute the mean value. */
	for (i = 0; i < FOCUS_REGIONS; i++)
		total += status_.focus_measures[i];
	mean = total / FOCUS_REGIONS;

	/* Compute the sum of the squared variance. */
	for (i = 0; i < FOCUS_REGIONS; i++)
		var_sum += std::pow(status_.focus_measures[i] - mean, 2);

	return var_sum / FOCUS_REGIONS;
}

bool Focus::afNeedIgnoreFrame()
{
	if (ignoreCounter_ == 0)
		return false;
	else
		ignoreCounter_--;
	return true;
}

void Focus::afCoarseScan()
{
	if (coarseCompleted_)
		return;

	if (afNeedIgnoreFrame())
		return;

	if (afScan(kCoarseSearchStep)) {
		coarseCompleted_ = true;
		status_.maxVariance = 0;
		focus_ = status_.focus - (status_.focus * kFineRange);
		status_.focus = focus_;
		previousVariance_ = 0;
		maxStep_ = std::clamp(focus_ + static_cast<uint32_t>((focus_ * kFineRange)),
				      0U, kMaxFocusSteps);
	}
}

void Focus::afFineScan()
{
	if (!coarseCompleted_)
		return;

	if (afNeedIgnoreFrame())
		return;

	if (afScan(kFineSearchStep)) {
		status_.stable = true;
		fineCompleted_ = true;
	}
}

bool Focus::afScan(uint32_t minSteps)
{
	if (focus_ > maxStep_) {
		/* If the max step is reached, move lens to the position. */
		status_.focus = bestFocus_;
		return true;
	} else {
		/*
		 * Find the maximum of the variance by estimating its
		 * derivative. If the direction changes, it means we have passed
		 * a maximum one step before.
		 */
		if ((currentVariance_ - status_.maxVariance) >=
		    -(status_.maxVariance * 0.1)) {
			/*
			 * Positive and zero derivative:
			 * The variance is still increasing. The focus could be
			 * increased for the next comparison. Also, the max
			 * variance and previous focus value are updated.
			 */
			bestFocus_ = focus_;
			focus_ += minSteps;
			status_.focus = focus_;
			status_.maxVariance = currentVariance_;
		} else {
			/*
			 * Negative derivative:
			 * The variance starts to decrease which means the maximum
			 * variance is found. Set focus step to previous good one
			 * then return immediately.
			 */
			status_.focus = bestFocus_;
			return true;
		}
	}

	previousVariance_ = currentVariance_;
	LOG(IoBFocus, Debug) << " Previous step is "
			     << bestFocus_
			     << " Current step is "
			     << focus_;
	return false;
}

void Focus::afReset()
{
	if (afNeedIgnoreFrame())
		return;

	status_.maxVariance = 0;
	status_.focus = 0;
	focus_ = 0;
	status_.stable = false;
	ignoreCounter_ = kIgnoreFrame;
	previousVariance_ = 0.0;
	coarseCompleted_ = false;
	fineCompleted_ = false;
	maxStep_ = kMaxFocusSteps;
}

bool Focus::afIsOutOfFocus()
{
	const uint32_t diff_var = std::abs(currentVariance_ -
					   status_.maxVariance);
	const double var_ratio = diff_var / status_.maxVariance;
	LOG(IoBFocus, Debug) << "Variance change rate: "
			     << var_ratio
			     << " Current VCM step: "
			     << status_.focus;
	if (var_ratio > kMaxChange)
		return true;
	else
		return false;
}

void Focus::Process(StatisticsPtr &stats, Metadata *image_metadata)
{
	unsigned int i;
	image_metadata->Get("focus.status", status_);

	/* Use the second filter results only, and cache those. */
	for (i = 0; i < FOCUS_REGIONS; i++)
		status_.focus_measures[i] = stats->focus_stats[i].contrast_val[1][1]
					  / stats->focus_stats[i].contrast_val_num[1][1];
	status_.num = i;

	currentVariance_ = estimateVariance();

	if (!status_.stable) {
		afCoarseScan();
		afFineScan();
	} else {
		if (afIsOutOfFocus())
			afReset();
		else
			ignoreCounter_ = kIgnoreFrame;
	}
}

/* Register algorithm with the system. */
static Algorithm *Create(Controller *controller)
{
	return new Focus(controller);
}
static RegisterAlgorithm reg(NAME, &Create);
