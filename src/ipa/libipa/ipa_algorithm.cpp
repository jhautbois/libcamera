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

// For registering algorithms with the system:

static std::map<std::string, AlgoCreateFunc> algorithms;
std::map<std::string, AlgoCreateFunc> const &GetAlgorithms()
{
	return algorithms;
}

RegisterAlgorithm::RegisterAlgorithm(char const *name,
				     AlgoCreateFunc create_func)
{
	algorithms[std::string(name)] = create_func;
}
