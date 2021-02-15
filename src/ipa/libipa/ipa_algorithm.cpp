/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * ipa_algorithm.cpp - ISP control algorithms
 */
#include <iostream>

#include <libcamera/ipa/ipa_algorithm.h>

using namespace libcamera;

void IPAAlgorithm::Initialise()
{
	std::cout << "Entering: " << __func__ << std::endl;
}

void IPAAlgorithm::Prepare() {}

void IPAAlgorithm::Process() {}
