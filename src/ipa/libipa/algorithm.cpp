/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * algorithm.cpp - ISP control algorithms
 */

#include "algorithm.h"

/**
 * \file algorithm.h
 * \brief Algorithm common interface
 */

namespace libcamera {

namespace ipa {

void Algorithm::initialise() {}

void Algorithm::process() {}

} /* namespace ipa */

} /* namespace libcamera */
