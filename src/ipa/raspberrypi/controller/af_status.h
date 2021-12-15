/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2022, Raspberry Pi (Trading) Limited
 * Copyright (C) 2022, Ideas On Board
 *
 * af_status.h - autofocus measurement status
 */
#pragma once

#include <libcamera/geometry.h>

/*
 * The focus algorithm should post the following structure into the image's
 * "af.status" metadata.
 */

struct AfStatus {
	uint32_t lensPosition;
	uint32_t state;
	libcamera::Rectangle windows;
};