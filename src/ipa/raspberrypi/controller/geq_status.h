/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * geq_status.h - GEQ (green equalisation) control algorithm status
 */
#pragma once

// The "GEQ" algorithm calculates the green equalisation thresholds

struct GeqStatus {
	uint16_t offset;
	double slope;
};
