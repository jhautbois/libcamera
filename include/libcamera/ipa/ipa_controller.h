/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * ipa_controller.h - ISP controller interface
 */
#ifndef __LIBCAMERA_IPA_CONTROLLER_H__
#define __LIBCAMERA_IPA_CONTROLLER_H__

// The Controller is simply a container for a collecting together a number of
// "control algorithms" (such as AWB etc.) and for running them all in a
// convenient manner.

#include <vector>
#include <string>

namespace libcamera {

class IPAAlgorithm;
typedef std::unique_ptr<IPAAlgorithm> IPAAlgorithmPtr;

class IPAController
{
public:
	IPAController();
	~IPAController();
	IPAAlgorithm *CreateAlgorithm(char const *name);
	void Initialise();
	void Prepare();
	void Process();
	IPAAlgorithm *GetAlgorithm(std::string const &name) const;

protected:
	std::vector<IPAAlgorithmPtr> algorithms_;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_IPA_CONTROLLER_H__ */
