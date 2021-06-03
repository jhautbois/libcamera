/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021 Ideas On Board
 *
 * camera_sensor_helper.cpp - helper class providing camera informations
 */
#include "camera_sensor_helper.h"

#include <map>

#include "libcamera/internal/log.h"

/**
 * \file camera_sensor_helper.h
 * \brief Helper class providing camera sensor informations
 */

/**
 * \file camera_sensor_helper.h
 * \brief Create helper class for each sensor
 *
 * Each camera sensor supported by libcamera may need specific informations to
 * be sent (analogue gain is sensor dependant for instance).
 *
 * Every subclass of CameraSensorHelper shall be registered with libipa using
 * the REGISTER_CAMERA_SENSOR_HELPER() macro.
 */

namespace libcamera {

LOG_DEFINE_CATEGORY(CameraSensorHelper)

namespace ipa {

/**
 * \class CameraSensorHelper
 * \brief Create and give helpers for each camera sensor in the pipeline
 *
 * CameraSensorHelper instances are unique and sensor dependant.
 */

/**
 * \brief Construct a CameraSensorHelper instance
 * \param[in] name The sensor model name
 *
 * The CameraSensorHelper instances shall never be constructed manually, but always
 * through the CameraSensorHelperFactory::create() method implemented by the
 * respective factories.
 */

CameraSensorHelper::CameraSensorHelper(const char *name)
	: name_(name)
{
	analogueGain_ = { GlobalGain, 1, 1, 0, 0 };
}

CameraSensorHelper::~CameraSensorHelper()
{
}

/**
 * \fn CameraSensorHelper::getGainCode(double gain)
 * \brief Get the analogue gain code to pass to V4L2 subdev control
 * \param[in] gain The real gain to pass
 *
 * This function aims to abstract the calculation of the gain letting the IPA
 * use the real gain for its estimations.
 *
 * The parameters come from the MIPI Alliance Camera Specification for
 * Camera Command Set (CCS).
 */
uint32_t CameraSensorHelper::getGainCode(double gain) const
{
	/* \todo we only support the global gain mode for now */
	if (analogueGain_.type != GlobalGain)
		return UINT32_MAX;

	if (analogueGain_.m0 == 0)
		return (analogueGain_.c0 - gain * analogueGain_.c1) / (gain * analogueGain_.m1);
	if (analogueGain_.m1 == 0)
		return (gain * analogueGain_.c1 - analogueGain_.c0) / analogueGain_.m0;

	LOG(CameraSensorHelper, Error) << "For any given image sensor either m0 or m1 shall be zero.";
	return 1;
}

/**
 * \fn CameraSensorHelper::getGain
 * \brief Get the real gain from the V4l2 subdev control gain
 * \param[in] gain The V4l2 subdev control gain
 *
 * This function aims to abstract the calculation of the gain letting the IPA
 * use the real gain for its estimations. It is the counterpart of the function
 * CameraSensorHelper::getGainCode.
 *
 * The parameters come from the MIPI Alliance Camera Specification for
 * Camera Command Set (CCS).
 */
double CameraSensorHelper::getGain(uint32_t gainCode) const
{
	if (analogueGain_.type != GlobalGain)
		return UINT32_MAX;

	if (analogueGain_.m0 == 0)
		return analogueGain_.c0 / (analogueGain_.m1 * gainCode + analogueGain_.c1);
	if (analogueGain_.m1 == 0)
		return (analogueGain_.m0 * gainCode + analogueGain_.c0) / analogueGain_.c1;

	LOG(CameraSensorHelper, Error) << "For any given image sensor either m0 or m1 shall be zero.";
	return 1.0;
}

/**
 * \fn CameraSensorHelper::name()
 * \brief Retrieve the camera sensor helper name
 * \return The camera sensor helper name
 */

/**
 * \class CameraSensorHelperFactory
 * \brief Registration of CameraSensorHelperFactory classes and creation of instances
 *
 * To facilitate discovery and instantiation of CameraSensorHelper classes, the
 * CameraSensorHelperFactory class maintains a registry of camera sensor helper
 * classes. Each CameraSensorHelper subclass shall register itself using the
 * REGISTER_CAMERA_SENSOR_HELPER() macro, which will create a corresponding
 * instance of a CameraSensorHelperFactory subclass and register it with the
 * static list of factories.
 */

/**
 * \brief Construct a camera sensor helper factory
 * \param[in] name Name of the camera sensor helper class
 *
 * Creating an instance of the factory registers it with the global list of
 * factories, accessible through the factories() function.
 *
 * The factory \a name is used for debug purpose and shall be unique.
 */

CameraSensorHelperFactory::CameraSensorHelperFactory(const char *name)
	: name_(name)
{
	registerType(this);
}

/**
 * \brief Create an instance of the CameraSensorHelper corresponding to the factory
 *
 * \return A unique pointer to a new instance of the CameraSensorHelper subclass
 * corresponding to the factory
 */
std::unique_ptr<CameraSensorHelper> CameraSensorHelperFactory::create()
{
	CameraSensorHelper *handler = createInstance();
	return std::unique_ptr<CameraSensorHelper>(handler);
}

/**
 * \fn CameraSensorHelperFactory::name()
 * \brief Retrieve the factory name
 * \return The factory name
 */

/**
 * \brief Add a camera sensor helper class to the registry
 * \param[in] factory Factory to use to construct the camera sensor helper
 *
 * The caller is responsible to guarantee the uniqueness of the camera sensor helper
 * name.
 */
void CameraSensorHelperFactory::registerType(CameraSensorHelperFactory *factory)
{
	std::vector<CameraSensorHelperFactory *> &factories = CameraSensorHelperFactory::factories();

	factories.push_back(factory);
}

/**
 * \brief Retrieve the list of all camera sensor helper factories
 *
 * The static factories map is defined inside the function to ensures it gets
 * initialized on first use, without any dependency on link order.
 *
 * \return The list of camera sensor helper factories
 */
std::vector<CameraSensorHelperFactory *> &CameraSensorHelperFactory::factories()
{
	static std::vector<CameraSensorHelperFactory *> factories;
	return factories;
}

/**
 * \fn CameraSensorHelperFactory::createInstance()
 * \brief Create an instance of the CameraSensorHelper corresponding to the factory
 *
 * This virtual function is implemented by the REGISTER_CAMERA_SENSOR_HELPER()
 * macro. It creates a camera sensor helper instance associated with the camera
 * sensor model.
 *
 * \return A pointer to a newly constructed instance of the CameraSensorHelper
 * subclass corresponding to the factory
 */

/**
 * \def REGISTER_CAMERA_SENSOR_HELPER
 * \brief Register a camera sensor helper with the camera sensor helper factory
 * \param[in] name Sensor model name used to register the class
 * \param[in] handler Class name of CameraSensorHelper derived class to register
 *
 * Register a CameraSensorHelper subclass with the factory and make it available to
 * try and match devices.
 */

/**
 * \class CameraSensorHelperImx219
 * \brief Create and give helpers for the imx219 sensor
 */
class CameraSensorHelperImx219 : public CameraSensorHelper
{
public:
	CameraSensorHelperImx219(const char *name);
};
REGISTER_CAMERA_SENSOR_HELPER("imx219", CameraSensorHelperImx219)
CameraSensorHelperImx219::CameraSensorHelperImx219(const char *name)
	: CameraSensorHelper(name)
{
	analogueGain_ = { GlobalGain, 0, -1, 256, 256 };
}

/**
 * \class CameraSensorHelperOv5670
 * \brief Create and give helpers for the ov5670 sensor
 */
class CameraSensorHelperOv5670 : public CameraSensorHelper
{
public:
	CameraSensorHelperOv5670(const char *name);
};
REGISTER_CAMERA_SENSOR_HELPER("ov5670", CameraSensorHelperOv5670)
CameraSensorHelperOv5670::CameraSensorHelperOv5670(const char *name)
	: CameraSensorHelper(name)
{
	analogueGain_ = { GlobalGain, 1, 0, 0, 256 };
}

/**
 * \class CameraSensorHelperOv5693
 * \brief Create and give helpers for the ov5693 sensor
 */
class CameraSensorHelperOv5693 : public CameraSensorHelper
{
public:
	CameraSensorHelperOv5693(const char *name);
};
REGISTER_CAMERA_SENSOR_HELPER("ov5693", CameraSensorHelperOv5693)
CameraSensorHelperOv5693::CameraSensorHelperOv5693(const char *name)
	: CameraSensorHelper(name)
{
	analogueGain_ = { GlobalGain, 1, 0, 0, 16 };
}

/**
 * \enum CameraSensorHelper::AnalogueGainCapability
 * \brief Specify the Gain mode supported by the sensor
 * \var CameraSensorHelper::AnalogueGainCapability::GlobalGain
 * \brief Sensor supports Global gain
 * \var CameraSensorHelper::AnalogueGainCapability::AlternateGlobalGain
 * \brief Sensor supports Analogue Global gain (introduced in CCS v1.1)
 */
/*
struct analogueGainConstants {
	uint16_t type;
	int16_t m0;
	int16_t c0;
	int16_t m1;
	int16_t c1;
};*/

} /* namespace ipa */

} /* namespace libcamera */

