/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021 Ideas On Board
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

class CameraSensorHelper : public Object
{
public:
	CameraSensorHelper(const char *name);
	virtual ~CameraSensorHelper();

	const std::string &name() const { return name_; }
	uint32_t getGainCode(double gain) const;
	double getGain(uint32_t gainCode) const;

protected:
	enum AnalogueGainCapability : uint16_t {
		GlobalGain = 0,
		AlternateGlobalGain = 2,
	};

	struct analogueGainConstants {
		uint16_t type;
		int16_t m0;
		int16_t c0;
		int16_t m1;
		int16_t c1;
	};
	analogueGainConstants analogueGain_;

private:
	std::string name_;
	friend class CameraSensorHelperFactory;
};

class CameraSensorHelperFactory
{
public:
	CameraSensorHelperFactory(const char *name);
	virtual ~CameraSensorHelperFactory() = default;

	std::unique_ptr<CameraSensorHelper> create();

	const std::string &name() const { return name_; }

	static void registerType(CameraSensorHelperFactory *factory);
	static std::vector<CameraSensorHelperFactory *> &factories();

private:
	virtual CameraSensorHelper *createInstance() = 0;

	std::string name_;
};

#define REGISTER_CAMERA_SENSOR_HELPER(name, handler)                        \
	class handler##Factory final : public CameraSensorHelperFactory     \
	{                                                                   \
	public:                                                             \
		handler##Factory() : CameraSensorHelperFactory(#handler) {} \
                                                                            \
	private:                                                            \
		CameraSensorHelper *createInstance()                        \
		{                                                           \
			return new handler(name);                           \
		}                                                           \
	};                                                                  \
	static handler##Factory global_##handler##Factory;

} /* namespace ipa */

} /* namespace libcamera */

#endif /* __LIBCAMERA_IPA_LIBIPA_CAMERA_SENSOR_HELPER_H__ */

