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
	LOG(IPAController, Error) << "Create algorithm " << name;
	return nullptr;
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
