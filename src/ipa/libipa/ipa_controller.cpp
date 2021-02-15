/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * ipa_controller.cpp - ISP controller
 */

#include "libcamera/internal/log.h"

#include <libcamera/ipa/ipa_algorithm.h>
#include <libcamera/ipa/ipa_controller.h>

using namespace libcamera;

LOG_DEFINE_CATEGORY(IPAController)

IPAController::IPAController() {}

IPAController::~IPAController() {}

IPAAlgorithm *IPAController::CreateAlgorithm(char const *name)
{
	LOG(IPAController, Error) << "Entering " << __func__;
	auto it = GetAlgorithms().find(std::string(name));
	return it != GetAlgorithms().end() ? (*it->second)(this) : nullptr;
}

void IPAController::Initialise()
{
	for (auto &algo : algorithms_)
		algo->Initialise();
}

void IPAController::Prepare()
{
	for (auto &algo : algorithms_)
		if (!algo->IsPaused())
			algo->Prepare();
}

void IPAController::Process()
{
	for (auto &algo : algorithms_)
		if (!algo->IsPaused())
			algo->Process();
}

IPAAlgorithm *IPAController::GetAlgorithm(std::string const &name) const
{
	LOG(IPAController, Error) << "Entering " << __func__;
	// The passed name must be the entire algorithm name, or must match the
	// last part of it with a period (.) just before.
	size_t name_len = name.length();
	for (auto &algo : algorithms_) {
		char const *algo_name = algo->Name();
		size_t algo_name_len = strlen(algo_name);
		if (algo_name_len >= name_len &&
		    strcasecmp(name.c_str(),
			       algo_name + algo_name_len - name_len) == 0 &&
		    (name_len == algo_name_len ||
		     algo_name[algo_name_len - name_len - 1] == '.'))
			return algo.get();
	}
	return nullptr;
}
