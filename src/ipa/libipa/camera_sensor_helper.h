/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Google Inc.
 *
 * camera_sensor_helper.h - helper class providing camera informations
 */
#ifndef __LIBCAMERA_IPA_LIBIPA_CAMERA_SENSOR_HELPER_H__
#define __LIBCAMERA_IPA_LIBIPA_CAMERA_SENSOR_HELPER_H__

#include <stdint.h>

#include <string>

#include <libcamera/class.h>
#include <libcamera/object.h>

namespace libcamera {

namespace ipa {

class CameraSensorHelper;

class CameraSensorHelper
{
public:
	CameraSensorHelper(const std::string name);
	virtual ~CameraSensorHelper();

	uint32_t GainCode(double gain) const;
	double Gain(uint32_t gainCode) const;

protected:
	enum AnalogueGainType {
		AnalogueGainLinear = 0,
		AnalogueGainExponential = 2,
	};

	struct AnalogueGainConstants {
		uint16_t type;
		int16_t m0;
		int16_t c0;
		int16_t m1;
		int16_t c1;
	};

	AnalogueGainConstants analogueGainConstants_;

private:
	LIBCAMERA_DISABLE_COPY_AND_MOVE(CameraSensorHelper)
	std::string name_;
};

class CameraSensorHelperFactory
{
public:
	CameraSensorHelperFactory(const std::string name);
	virtual ~CameraSensorHelperFactory() = default;

	static std::unique_ptr<CameraSensorHelper> create(const std::string &name);

	static void registerType(CameraSensorHelperFactory *factory);
	static std::vector<CameraSensorHelperFactory *> &factories();

private:
	LIBCAMERA_DISABLE_COPY_AND_MOVE(CameraSensorHelperFactory)
	virtual CameraSensorHelper *createInstance() = 0;

	std::string name_;
};

#define REGISTER_CAMERA_SENSOR_HELPER(name, helper)               \
class helper##Factory final : public CameraSensorHelperFactory    \
{                                                                 \
public:                                                           \
	helper##Factory() : CameraSensorHelperFactory(name) {} \
								  \
private:                                                          \
	CameraSensorHelper *createInstance()                      \
	{                                                         \
		return new helper(name);                          \
	}                                                         \
};                                                                \
static helper##Factory global_##helper##Factory;

} /* namespace ipa */

} /* namespace libcamera */

#endif /* __LIBCAMERA_IPA_LIBIPA_CAMERA_SENSOR_HELPER_H__ */

