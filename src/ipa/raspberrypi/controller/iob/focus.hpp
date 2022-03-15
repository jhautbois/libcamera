/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Red Hat
 * Copyright (C) 2022, Ideas On Board
 *
 * focus.hpp - focus algorithm
 */
#pragma once

#include "../algorithm.hpp"
#include "../metadata.hpp"

/*
 * The "focus" algorithm. All it does it print out a version of the
 * focus contrast measure; there is no actual auto-focus mechanism to
 * control.
 */

namespace RPiController {

class Focus : public Algorithm
{
public:
	Focus(Controller *controller);
	char const *Name() const override;
	void Initialise() override;
	void Prepare(Metadata *image_metadata) override;
	void Process(StatisticsPtr &stats, Metadata *image_metadata) override;
private:
	double estimateVariance();
	bool afNeedIgnoreFrame();
	void afCoarseScan();
	void afFineScan();
	bool afScan(uint32_t minSteps);
	void afReset();
	bool afIsOutOfFocus();

	FocusStatus status_;

	/* VCM step configuration. It is the current setting of the VCM step. */
	uint32_t focus_;
	/* The best VCM step. It is a local optimum VCM step during scanning. */
	uint32_t bestFocus_;

	/* The frames ignored before starting measuring. */
	uint32_t ignoreCounter_;

	/* Current AF statistic variance. */
	double currentVariance_;
	/* It is used to determine the derivative during scanning */
	double previousVariance_;
	/* The designated maximum range of focus scanning. */
	uint32_t maxStep_;
	/* If the coarse scan completes, it is set to true. */
	bool coarseCompleted_;
	/* If the fine scan completes, it is set to true. */
	bool fineCompleted_;
};

} /* namespace RPiController */
