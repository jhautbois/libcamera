/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Ideas On Board
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
