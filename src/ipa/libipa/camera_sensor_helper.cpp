/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Google Inc.
 *
 * camera_sensor_helper.cpp - helper class providing camera informations
 */
#include "camera_sensor_helper.h"

#include <map>

#include "libcamera/internal/log.h"

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
 * through the CameraSensorHelperFactory::create() method.
 */

CameraSensorHelper::CameraSensorHelper(const std::string name)
	: name_(name)
{
}

CameraSensorHelper::~CameraSensorHelper()
{
}

/**
 * \fn CameraSensorHelper::getGainCode(double gain)
 * \brief Get the analogue gain code to pass to V4L2 subdev control
 * \param[in] gain The real gain to pass
 * \return the gain code to pass to V4l2
 *
 * This function aims to abstract the calculation of the gain letting the IPA
 * use the real gain for its estimations.
 *
 * The parameters come from the MIPI Alliance Camera Specification for
 * Camera Command Set (CCS).
 */
uint32_t CameraSensorHelper::GainCode(double gain) const
{
	ASSERT((analogueGainConstants_.m0 == 0) || (analogueGainConstants_.m1 == 0));
	ASSERT(analogueGainConstants_.type == AnalogueGainLinear);
	
	return (analogueGainConstants_.c0 - analogueGainConstants_.c1 * gain) /
	       (analogueGainConstants_.m1 * gain - analogueGainConstants_.m0);
}

/**
 * \fn CameraSensorHelper::getGain
 * \brief Get the real gain from the V4l2 subdev control gain
 * \param[in] gainCode The V4l2 subdev control gain
 * \return The real gain
 *
 * This function aims to abstract the calculation of the gain letting the IPA
 * use the real gain for its estimations. It is the counterpart of the function
 * CameraSensorHelper::getGainCode.
 *
 * The parameters come from the MIPI Alliance Camera Specification for
 * Camera Command Set (CCS).
 */
double CameraSensorHelper::Gain(uint32_t gainCode) const
{
	ASSERT((analogueGainConstants_.m0 == 0) || (analogueGainConstants_.m1 == 0));
	ASSERT(analogueGainConstants_.type == AnalogueGainLinear);

	return (analogueGainConstants_.m0 * gainCode + analogueGainConstants_.c0) /
	       (analogueGainConstants_.m1 * gainCode + analogueGainConstants_.c1);
}

/**
 * \fn CameraSensorHelper::name()
 * \brief Retrieve the camera sensor helper name
 * \return The camera sensor helper name
 */

/**
 * \enum CameraSensorHelper::AnalogueGainType
 * \brief Specify the Gain mode supported by the sensor
 *
 * Describes the image sensor analog Gain capabilities.
 * Two modes are possible, depending on the sensor: Global and Alternate.
 */

/**
 * \var CameraSensorHelper::GlobalGain
 * \brief Sensor supports Global gain
 *
 * The relationship between the integer Gain parameter and the resulting Gain
 * multiplier is given by the following equation:
 *
 * \fgain=$\frac{m0x+c0}{m1x+c1}\f$
 *
 * Where 'x' is the gain control parameter, and m0, m1, c0 and c1 are
 * image-sensor-specific constants exposed by the sensor.
 * These constants are static parameters, and for any given image sensor either
 * m0 or m1 shall be zero.
 *
 * The full Gain equation therefore reduces to either:
 *
 * \f$ gain=\frac{c0}{m1x+c1}\f$ or \f$\frac{m0x+c0}{c1} \f$
 */

/**
 * \var CameraSensorHelper::AlternateGlobalGain
 * \brief Sensor supports Analogue Global gain (introduced in CCS v1.1)
 *
 * Starting with CCS v1.1, Alternate Global Analog Gain is also available.
 * If the image sensor supports it, then the global analog Gain can be controlled
 * by linear and exponential gain formula:
 *
 * \f$ gain = analogLinearGainGlobal * 2^{analogExponentialGainGlobal} \f$
 * \todo not implemented in libipa
 */

/**
 * \struct CameraSensorHelper::analogueGainConstants
 * \brief Analogue gain constants used for gain calculation
 */

/**
 * \var CameraSensorHelper::analogueGainConstants::type
 * \brief Analogue gain coding type
 */

/**
 * \var CameraSensorHelper::analogueGainConstants::m0
 * \brief Constant used in the analog Gain control coding/decoding.
 *
 * Note: either m0 or m1 shall be zero.
 */

/**
 * \var CameraSensorHelper::analogueGainConstants::c0
 * \brief Constant used in the analog Gain control coding/decoding.
 */

/**
 * \var CameraSensorHelper::analogueGainConstants::m1
 * \brief Constant used in the analog Gain control coding/decoding.
 *
 * Note: either m0 or m1 shall be zero.
 */

/**
 * \var CameraSensorHelper::analogueGainConstants::c1
 * \brief Constant used in the analog Gain control coding/decoding.
 */

/**
 * \var CameraSensorHelper::analogueGain_
 * \brief The analogue gain parameters used for calculation
 *
 * The analogue gain is calculated through a formula, and its parameters are
 * sensor specific. Use this variable to store the values at init time.
 *
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

CameraSensorHelperFactory::CameraSensorHelperFactory(const std::string name)
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
std::unique_ptr<CameraSensorHelper> CameraSensorHelperFactory::create(const std::string &name)
{
	std::vector<CameraSensorHelperFactory *> &factories =
		CameraSensorHelperFactory::factories();

	for (CameraSensorHelperFactory *factory : factories) {
		if (name != factory->name_)
			continue;

		CameraSensorHelper *helper = factory->createInstance();
		return std::unique_ptr<CameraSensorHelper>(helper);
	}
	return nullptr;
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

/**
 * \fn CameraSensorHelperImx219::CameraSensorHelperImx219
 * \brief Construct a CameraSensorHelperImx219 instance for imx219
 * \param[in] name The sensor model name
 */
class CameraSensorHelperImx219 : public CameraSensorHelper
{
public:
	CameraSensorHelperImx219(const std::string name);
};
REGISTER_CAMERA_SENSOR_HELPER("imx219", CameraSensorHelperImx219)
CameraSensorHelperImx219::CameraSensorHelperImx219(const std::string name)
	: CameraSensorHelper(name)
{
	analogueGainConstants_ = { AnalogueGainLinear, 0, -1, 256, 256 };
}

/**
 * \class CameraSensorHelperOv5670
 * \brief Create and give helpers for the ov5670 sensor
 */

/**
 * \fn CameraSensorHelperOv5670::CameraSensorHelperOv5670
 * \brief Construct a CameraSensorHelperOv5670 instance for ov5670
 * \param[in] name The sensor model name
 */
class CameraSensorHelperOv5670 : public CameraSensorHelper
{
public:
	CameraSensorHelperOv5670(const std::string name);
};
REGISTER_CAMERA_SENSOR_HELPER("ov5670", CameraSensorHelperOv5670)
CameraSensorHelperOv5670::CameraSensorHelperOv5670(const std::string name)
	: CameraSensorHelper(name)
{
	analogueGainConstants_ = { AnalogueGainLinear, 1, 0, 0, 256 };
}

/**
 * \class CameraSensorHelperOv5693
 * \brief Create and give helpers for the ov5693 sensor
 */

/**
 * \fn CameraSensorHelperOv5693::CameraSensorHelperOv5693
 * \brief Construct a CameraSensorHelperOv5693 instance for ov5693
 * \param[in] name The sensor model name
 */
class CameraSensorHelperOv5693 : public CameraSensorHelper
{
public:
	CameraSensorHelperOv5693(const std::string name);
};
REGISTER_CAMERA_SENSOR_HELPER("ov5693", CameraSensorHelperOv5693)
CameraSensorHelperOv5693::CameraSensorHelperOv5693(const std::string name)
	: CameraSensorHelper(name)
{
	analogueGainConstants_ = { AnalogueGainLinear, 1, 0, 0, 16 };
}

} /* namespace ipa */

} /* namespace libcamera */

