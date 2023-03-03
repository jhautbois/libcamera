/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Collabora Ltd.
 *     Author: Nicolas Dufresne <nicolas.dufresne@collabora.com>
 *
 * gstlibcamera-controls.cpp - GStreamer Camera Controls
 */

#include "gstlibcamera-controls.h"

#include <libcamera/control_ids.h>

using namespace libcamera;

#define AE_CONSTRAINT_MODE (ae_constraint_mode_get_type())
static const GEnumValue ae_constraint_modes[] = {
	{controls::ConstraintNormal, "Default constraint mode. "
		"This mode aims to balance the exposure of different parts of the "
		"image so as to reach a reasonable average level. However, highlights "
		"in the image may appear over-exposed and lowlights may appear "
		"under-exposed", "normal"},
	{controls::ConstraintHighlight,
		"Highlight constraint mode. "
		"This mode adjusts the exposure levels in order to try and avoid "
		"over-exposing the brightest parts (highlights) of an image. "
		"Other non-highlight parts of the image may appear under-exposed.",
		"highlight"},
	{controls::ConstraintShadows,
		"Shadows constraint mode. "
		"This mode adjusts the exposure levels in order to try and avoid "
		"under-exposing the dark parts (shadows) of an image. Other normally "
		"exposed parts of the image may appear over-exposed.",
		"shadows"},
	{controls::ConstraintCustom, "Custom constraint mode", "custom"},
	{0, NULL, NULL},
};

static GType
ae_constraint_mode_get_type (void)
{
	static GType ae_constraint_mode_type = 0;

	if (!ae_constraint_mode_type) {
		ae_constraint_mode_type =
			g_enum_register_static ("AeConstraintMode",
						ae_constraint_modes);
	}
	return ae_constraint_mode_type;
}

#define AE_EXPOSURE_MODE (ae_exposure_mode_get_type())
static const GEnumValue ae_exposure_modes[] = {
	{controls::ExposureNormal, "Default exposure mode", "normal"},
	{controls::ExposureShort,
		"Exposure mode allowing only short exposure times",
		"short"},
	{controls::ExposureLong,
		"Exposure mode allowing long exposure times", "long"},
	{controls::ExposureCustom, "Custom exposure mode", "custom"},
	{0, NULL, NULL},
};

static GType
ae_exposure_mode_get_type (void)
{
	static GType ae_exposure_mode_type = 0;

	if (!ae_exposure_mode_type) {
		ae_exposure_mode_type =
			g_enum_register_static ("AeExposureMode",
						ae_exposure_modes);
	}
	return ae_exposure_mode_type;
}

#define AE_METERING_MODE (ae_metering_mode_get_type())
static const GEnumValue ae_metering_modes[] = {
	{controls::MeteringCentreWeighted, "Centre-weighted metering mode",
		"centre-weighted"},
	{controls::MeteringSpot, "Spot metering mode", "spot"},
	{controls::MeteringMatrix, "Matrix metering mode", "matrix"},
	{controls::MeteringCustom, "Custom metering mode", "custom"},
	{0, NULL, NULL},
};

static GType
ae_metering_mode_get_type (void)
{
	static GType ae_metering_mode_type = 0;

	if (!ae_metering_mode_type) {
		ae_metering_mode_type =
			g_enum_register_static ("AeMeteringMode",
						ae_metering_modes);
	}
	return ae_metering_mode_type;
}

#define AWB_MODE (awb_mode_get_type())
static const GEnumValue awb_modes[] = {
	{controls::AwbAuto,
		"Search over the whole colour temperature range", "auto"},
	{controls::AwbIncandescent, "Incandescent AWB lamp mode",
		"incandescent"},
	{controls::AwbTungsten, "Tungsten AWB lamp mode", "tungsten"},
	{controls::AwbFluorescent, "Fluorescent AWB lamp mode",
		"fluorescent"},
	{controls::AwbIndoor, "Indoor AWB lighting mode", "indoor"},
	{controls::AwbDaylight, "Daylight AWB lighting mode", "daylight"},
	{controls::AwbCloudy, "Cloudy AWB lighting mode", "cloudy"},
	{controls::AwbCustom, "Custom AWB mode", "custom"},
	{0, NULL, NULL},
};

static GType
awb_mode_get_type (void)
{
	static GType awb_mode_type = 0;

	if (!awb_mode_type) {
		awb_mode_type = g_enum_register_static ("AwbMode",
							awb_modes);
	}
	return awb_mode_type;
}

#define NOISE_REDUCTION_MODE (noise_reduction_mode_get_type())
static const GEnumValue noise_reduction_modes[] = {
	{controls::draft::NoiseReductionModeOff, "No noise reduction is applied",
		"off"},
	{controls::draft::NoiseReductionModeFast,
		"Noise reduction is applied without reducing the frame rate",
		"fast"},
	{controls::draft::NoiseReductionModeHighQuality,
		"High quality noise reduction at the expense of frame rate",
		"high-quality"},
	{controls::draft::NoiseReductionModeMinimal,
		"Minimal noise reduction is applied without reducing the frame rate",
		"minimal"},
	{controls::draft::NoiseReductionModeZSL,
		"Noise reduction is applied at different levels to different streams",
		"zsl"},
	{0, NULL, NULL},
};

static GType
noise_reduction_mode_get_type (void)
{
	static GType noise_reduction_mode_type = 0;

	if (!noise_reduction_mode_type) {
	noise_reduction_mode_type =
		g_enum_register_static ("NoiseReductionMode",
					noise_reduction_modes);
	}
	return noise_reduction_mode_type;
}

void GstCameraControls::installProperties(GObjectClass *klass, int lastPropId)
{
	g_object_class_install_property (klass, lastPropId + AeAnalogueGain,
		g_param_spec_float("ae-analogue-gain", "AE Analogue Gain",
				   "Analogue gain value applied in the sensor device. "
				   "The value of the control specifies the gain multiplier applied to all "
				   "colour channels. This value cannot be lower than 1.0. "
				   "Setting this value means that it is now fixed and the AE algorithm may "
				   "not change it. Setting it back to zero returns it to the control of the "
				   "AE algorithm.",
				   0.0f, 16.0f, 1.0f,
				   (GParamFlags)(GST_PARAM_CONTROLLABLE
						 | G_PARAM_READWRITE
						 | G_PARAM_STATIC_STRINGS)));

	g_object_class_install_property (klass, lastPropId + AeExposureMode,
		g_param_spec_enum ("ae-constraint-mode", "Constraint Mode",
				   "Specify a constraint mode for the AE algorithm to use. These determine "
				   "how the measured scene brightness is adjusted to reach the desired "
				   "target exposure. Constraint modes may be platform specific, and not "
				   "all constraint modes may be supported.",
				   AE_CONSTRAINT_MODE,
				   controls::ConstraintNormal,
				   (GParamFlags)(GST_PARAM_CONTROLLABLE
						 | G_PARAM_READWRITE
						 | G_PARAM_STATIC_STRINGS)));

	g_object_class_install_property (klass, lastPropId + AeEnable,
		g_param_spec_boolean ("ae-enable",
				      "Auto Exposure",
				      "Enable or disable the Automatic Exposure algorithm.",
				      true,
				      (GParamFlags)(GST_PARAM_CONTROLLABLE
						    | G_PARAM_READWRITE
						    | G_PARAM_STATIC_STRINGS)));

	g_object_class_install_property (klass, lastPropId + AeExposureMode,
		g_param_spec_enum ("ae-exposure-mode", "Exposure Mode",
				   "Specify an exposure mode for the AE algorithm to use. These specify "
				   "how the desired total exposure is divided between the shutter time "
				   "and the sensor's analogue gain. The exposure modes are platform "
				   "specific, and not all exposure modes may be supported. ",
				   AE_EXPOSURE_MODE,
				   controls::ExposureNormal,
				   (GParamFlags)(GST_PARAM_CONTROLLABLE
						 | G_PARAM_READWRITE
						 | G_PARAM_STATIC_STRINGS)));

	g_object_class_install_property(klass, lastPropId + AeExposureTime,
		g_param_spec_int("ae-exposure-time", "Exposure time",
				 "Exposure time (shutter speed) for the frame applied in the sensor "
				 "device. This value is specified in micro-seconds. "
				 "Setting this value means that it is now fixed and the AE algorithm may "
				 "not change it. Setting it back to zero returns it to the control of the "
				 "AE algorithm.",
				 0, 66666, 0,
				 (GParamFlags)(GST_PARAM_CONTROLLABLE
					       | G_PARAM_READWRITE
					       | G_PARAM_STATIC_STRINGS)));

	g_object_class_install_property (klass, lastPropId + AeExposureValue,
		g_param_spec_float ("ae-exposure-value",
				    "Exposure Value",
				    "Specify an Exposure Value (EV) parameter. The EV parameter will only be "
				    "applied if the AE algorithm is currently enabled. "
				    "By convention EV adjusts the exposure as log2. For example "
				    "EV = [-2, -1, 0.5, 0, 0.5, 1, 2] results in an exposure adjustment "
				    "of [1/4x, 1/2x, 1/sqrt(2)x, 1x, sqrt(2)x, 2x, 4x].",
				    -8.0f, 8.0f, 0.0f,
				    (GParamFlags)(GST_PARAM_CONTROLLABLE
						  | G_PARAM_READWRITE
						  | G_PARAM_STATIC_STRINGS)));

	g_object_class_install_property (klass, lastPropId + AeMeteringMode,
		g_param_spec_enum ("ae-metering-mode", "Metering Mode",
				   "Specify a metering mode for the AE algorithm to use. The metering "
				   "modes determine which parts of the image are used to determine the "
				   "scene brightness. Metering modes may be platform specific and not "
				   "all metering modes may be supported.",
				   AE_METERING_MODE,
				   controls::MeteringCentreWeighted,
				   (GParamFlags)(GST_PARAM_CONTROLLABLE
						 | G_PARAM_READWRITE
						 | G_PARAM_STATIC_STRINGS)));

	g_object_class_install_property (klass, lastPropId + AwbColorGainBlue,
		g_param_spec_float("awb-gain-blue", "AWB Blue Gain",
				   "Manual AWB Gain for blue channel when awb-enable=0",
				   0.0f, 32.0f, 0.0f,
				   (GParamFlags)(GST_PARAM_CONTROLLABLE
						 | G_PARAM_READWRITE
						 | G_PARAM_STATIC_STRINGS)));

	g_object_class_install_property (klass, lastPropId + AwbColorGainRed,
		g_param_spec_float("awb-gain-red", "AWB Red Gain",""
				   "Manual AWB Gain for red channel when awb-enable=0",
				   0.0f, 32.0f, 0.0f,
				   (GParamFlags)(GST_PARAM_CONTROLLABLE
						 | G_PARAM_READWRITE
						 | G_PARAM_STATIC_STRINGS)));

	g_object_class_install_property (klass, lastPropId + AwbEnable,
		g_param_spec_boolean ("awb-enable",
				      "Enable Automatic White Balance",
				      "Enable or disable the AWB.",
				      true,
				      (GParamFlags)(GST_PARAM_CONTROLLABLE
						    | G_PARAM_READWRITE
						    | G_PARAM_STATIC_STRINGS)));

	g_object_class_install_property (klass, lastPropId + AwbMode,
		g_param_spec_enum ("awb-mode",
				   "Automatic White Balance Mode",
				   "Specify the range of illuminants to use for the AWB algorithm. The modes "
				   "supported are platform specific, and not all modes may be supported.",
				   AWB_MODE, controls::AwbAuto,
				   (GParamFlags)(GST_PARAM_CONTROLLABLE
						 | G_PARAM_READWRITE
						 | G_PARAM_STATIC_STRINGS)));

	g_object_class_install_property(klass, lastPropId + Brightness,
		g_param_spec_float("brightness", "Brightness",
				   "Specify a fixed brightness parameter. Positive values (up to 1.0) "
				   "produce brighter images; negative values (up to -1.0) produce darker "
				   "images and 0.0 leaves pixels unchanged.",
				   -1.0f, 1.0f, 0.0f,
				   (GParamFlags)(GST_PARAM_CONTROLLABLE
						 | G_PARAM_READWRITE
						 | G_PARAM_STATIC_STRINGS)));

	g_object_class_install_property(klass, lastPropId + Contrast,
		g_param_spec_float("contrast", "Contrast",
				   "Specify a fixed contrast parameter. Normal contrast is given by the "
				   "value 1.0; larger values produce images with more contrast.",
				   0.0f, 32.0f, 1.0f,
				   (GParamFlags)(GST_PARAM_CONTROLLABLE
						 | G_PARAM_READWRITE
						 | G_PARAM_STATIC_STRINGS)));

	g_object_class_install_property (klass, lastPropId + NoiseReductionMode,
		g_param_spec_enum ("noise-reduction-mode", "Noise Reduction Mode",
				   "Control to select the noise reduction algorithm mode. Currently "
				   "identical to ANDROID_NOISE_REDUCTION_MODE.",
				   NOISE_REDUCTION_MODE,
				   controls::draft::NoiseReductionModeOff,
				   (GParamFlags)(GST_PARAM_CONTROLLABLE
						 | G_PARAM_READWRITE
						 | G_PARAM_STATIC_STRINGS)));

	g_object_class_install_property(klass, lastPropId + Saturation,
		g_param_spec_float("saturation", "Saturation",
				   "Specify a fixed saturation parameter. Normal saturation is given by "
				   "the value 1.0; larger values produce more saturated colours; 0.0 "
				   "produces a greyscale image.",
				   0.0f, 32.0f, 1.0f,
				   (GParamFlags)(GST_PARAM_CONTROLLABLE
						 | G_PARAM_READWRITE
						 | G_PARAM_STATIC_STRINGS)));

	g_object_class_install_property(klass, lastPropId + Sharpness,
		g_param_spec_float("sharpness", "Sharpness",
				   "A value of 0.0 means no sharpening. The minimum value means "
				   "minimal sharpening, and shall be 0.0 unless the camera can't "
				   "disable sharpening completely. The default value shall give a "
				   "'reasonable' level of sharpening, suitable for most use cases. "
				   "The maximum value may apply extremely high levels of sharpening, "
				   "higher than anyone could reasonably want. Negative values are "
				   "not allowed. Note also that sharpening is not applied to raw "
				   "streams.",
				   0.0f, 16.0f, 1.0f,
				   (GParamFlags)(GST_PARAM_CONTROLLABLE
						 | G_PARAM_READWRITE
						 | G_PARAM_STATIC_STRINGS)));
}

bool GstCameraControls::getProperty(guint propId, GValue *value, GParamSpec *pspec)
{
	switch (propId) {
	case AeAnalogueGain: {
		auto val = controls_.get(controls::AnalogueGain);
		auto spec = G_PARAM_SPEC_FLOAT(pspec);
		g_value_set_float(value,
				  val.value_or(spec->default_value));
		return true;
	}
	case AeConstraintMode: {
		auto val = controls_.get(controls::AeConstraintMode);
		auto spec = G_PARAM_SPEC_ENUM(pspec);
		g_value_set_enum(value, val.value_or(spec->default_value));
		return true;
	}
	case AeEnable: {
		auto val = controls_.get(controls::AeEnable);
		auto spec = G_PARAM_SPEC_BOOLEAN(pspec);
		g_value_set_boolean(value,
				    val.value_or(spec->default_value));
		return true;
	}
	case AeExposureMode: {
		auto val = controls_.get(controls::AeExposureMode);
		auto spec = G_PARAM_SPEC_ENUM(pspec);
		g_value_set_enum(value, val.value_or(spec->default_value));
		return true;
	}
	case AeExposureTime: {
		auto val = controls_.get(controls::ExposureTime);
		auto spec = G_PARAM_SPEC_INT(pspec);
		g_value_set_int(value, val.value_or(spec->default_value));
		return true;
	}
	case AeExposureValue: {
		auto val = controls_.get(controls::ExposureValue);
		auto spec = G_PARAM_SPEC_FLOAT(pspec);
		g_value_set_float(value,
				  val.value_or(spec->default_value));
		return true;
	}
	case AeMeteringMode: {
		auto val = controls_.get(controls::AeMeteringMode);
		auto spec = G_PARAM_SPEC_ENUM(pspec);
		g_value_set_enum(value, val.value_or(spec->default_value));
		return true;
	}
	case AwbColorGainBlue: {
		auto spec = G_PARAM_SPEC_FLOAT(pspec);
		const auto &colourGains =
			controls_.get(controls::ColourGains);
		auto val = colourGains ?
			(*colourGains)[1] : spec->default_value;
		g_value_set_float(value, val);
		return true;
	}
	case AwbColorGainRed: {
		auto spec = G_PARAM_SPEC_FLOAT(pspec);
		const auto &colourGains =
			controls_.get(controls::ColourGains);
		auto val = colourGains ?
			(*colourGains)[0] : spec->default_value;
		g_value_set_float(value, val);
		return true;
	}
	case AwbEnable: {
		auto val = controls_.get(controls::AwbEnable);
		auto spec = G_PARAM_SPEC_BOOLEAN(pspec);
		g_value_set_boolean(value,
				    val.value_or(spec->default_value));
		return true;
	}
	case AwbMode: {
		auto val = controls_.get(controls::AwbMode);
		auto *spec = G_PARAM_SPEC_ENUM(pspec);
		g_value_set_enum(value,
				 val.value_or(spec->default_value));
		return true;
	}
	case Brightness: {
		auto val = controls_.get(controls::Brightness);
		auto spec = G_PARAM_SPEC_FLOAT(pspec);
		/* FIXME the default from the init controls*/
		g_value_set_float(value,
				  val.value_or(spec->default_value));
		return true;
	}
	case Contrast: {
		auto val = controls_.get(controls::Contrast);
		auto spec = G_PARAM_SPEC_FLOAT(pspec);
		g_value_set_float(value,
				  val.value_or(spec->default_value));
		return true;
	}
	case NoiseReductionMode: {
		auto val =
			controls_.get(controls::draft::NoiseReductionMode);
		auto spec = G_PARAM_SPEC_ENUM(pspec);
		g_value_set_enum(value, val.value_or(spec->default_value));
		return true;
	}
	case Saturation: {
		auto val = controls_.get(controls::Saturation);
		auto spec = G_PARAM_SPEC_FLOAT(pspec);
		g_value_set_float(value,
				  val.value_or(spec->default_value));
		return true;
	}
	case Sharpness: {
		auto val = controls_.get(controls::Sharpness);
		auto spec = G_PARAM_SPEC_FLOAT(pspec);
		g_value_set_float(value,
				  val.value_or(spec->default_value));
		return true;
	}
	default:
		return false;
	}
}

bool GstCameraControls::setProperty(guint propId, const GValue *value,
				    GParamSpec *pspec)
{
	switch (propId) {
	case AeAnalogueGain:
		controls_.set(controls::AnalogueGain,
			      g_value_get_float(value));
		return true;
	case AeConstraintMode:
		controls_.set(controls::AeConstraintMode,
			      g_value_get_int(value));
		return true;
	case AeEnable:
		controls_.set(controls::AeEnable,
			g_value_get_boolean(value));
		return true;
	case AeExposureMode:
		controls_.set(controls::AeExposureMode,
			      g_value_get_int(value));
		return true;
	case AeExposureTime:
		controls_.set(controls::ExposureTime,
			      g_value_get_int(value));
		return true;
	case AeExposureValue:
		controls_.set(controls::ExposureValue,
			      g_value_get_float(value));
		return true;
	case AeMeteringMode:
		controls_.set(controls::AeMeteringMode,
			      g_value_get_int(value));
		return true;
	case AwbColorGainBlue: {
		auto spec = G_PARAM_SPEC_FLOAT(pspec);
		const auto &colourGains =
			controls_.get(controls::ColourGains);
		auto red_val = colourGains ?
			(*colourGains)[0] : spec->default_value;
		controls_.set(controls::ColourGains,
			      {red_val, g_value_get_float(value)});
		return true;
	}
	case AwbColorGainRed: {
		auto spec = G_PARAM_SPEC_FLOAT(pspec);
		const auto &colourGains =
			controls_.get(controls::ColourGains);
		auto blue_val = colourGains ?
			(*colourGains)[1] : spec->default_value;
		controls_.set(controls::ColourGains,
			      {g_value_get_float(value), blue_val});
		return true;
	}
	case AwbEnable:
		controls_.set(controls::AwbEnable,
			      g_value_get_boolean(value));
		return true;
	case AwbMode:
		controls_.set(controls::AwbMode,
			      g_value_get_int(value));
		return true;
	case Brightness:
		controls_.set(controls::Brightness,
			      g_value_get_float(value));
		return true;
	case Contrast:
		controls_.set(controls::Contrast,
			      g_value_get_float(value));
		return true;
	case NoiseReductionMode:
		controls_.set(controls::draft::NoiseReductionMode,
			      g_value_get_int(value));
		return true;
	case Saturation:
		controls_.set(controls::Saturation,
			      g_value_get_float(value));
		return true;
	case Sharpness:
		controls_.set(controls::Sharpness,
			      g_value_get_float(value));
		return true;
	default:
		break;
	}

	return false;
}

void GstCameraControls::applyControls(std::unique_ptr<libcamera::Request> &request)
{
	request->controls().merge(controls_);
}
