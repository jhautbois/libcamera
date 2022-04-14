/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Red Hat
 * Copyright (C) 2022, Ideas On Board
 *
 * af.h - automatic contrast-based focus algorithm
 */
#pragma once

#include <libcamera/geometry.h>

#include "../af_algorithm.hpp"
#include "../af_status.h"
#include "../metadata.hpp"

namespace RPiController {

class Af : public AfAlgorithm
{
public:
	Af(Controller *controller);
	char const *Name() const override;
	void Initialise() override;
	void Prepare(Metadata *image_metadata) override;
	void Process(StatisticsPtr &stats, Metadata *image_metadata) override;
	void SetMode(const uint32_t &mode) override;
	void Trigger() override;
	void Cancel() override;
	void SetWindows(const libcamera::Rectangle &afWindows) override;
	void SetRange(const uint32_t &low, const uint32_t &high) override;
	void setSpeed(const uint32_t &speed) override;
private:
	bool afNeedIgnoreFrame();
	void afCoarseScan();
	void afFineScan();
	bool afScan(uint32_t minSteps);
	void afReset();
	bool afIsOutOfFocus();

	AfStatus status_;

	/* VCM step configuration. It is the current setting of the VCM step. */
	uint32_t focus_;
	/* The best VCM step. It is a local optimum VCM step during scanning. */
	uint32_t bestFocus_;

	/* Current AF statistic contrast. */
	double currentContrast_;
	/* It is used to determine the derivative during scanning */
	double previousContrast_;
	double maxContrast_;
	/* The designated maximum range of focus scanning. */
	uint32_t maxStep_;
	/* If the coarse scan completes, it is set to true. */
	bool coarseCompleted_;
	/* If the fine scan completes, it is set to true. */
	bool fineCompleted_;

	uint32_t mode_;

	uint32_t lowStep_;
	uint32_t highStep_;
};

} /* namespace RPiController */
