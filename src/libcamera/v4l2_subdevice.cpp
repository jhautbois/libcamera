/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * v4l2_subdevice.cpp - V4L2 Subdevice
 */

#include "libcamera/internal/v4l2_subdevice.h"

#include <fcntl.h>
#include <iomanip>
#include <regex>
#include <sstream>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <linux/media-bus-format.h>
#include <linux/v4l2-subdev.h>

#include <libcamera/geometry.h>

#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include "libcamera/internal/formats.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/media_object.h"

/**
 * \file v4l2_subdevice.h
 * \brief V4L2 Subdevice API
 */

namespace libcamera {

LOG_DECLARE_CATEGORY(V4L2)

namespace {

/*
 * \struct MediaBusFormatInfo
 * \brief Information about media bus formats
 * \param name Name of MBUS format
 * \param code The media bus format code
 * \param bitsPerPixel Bits per pixel
 * \param colourEncoding Type of colour encoding
 */
struct MediaBusFormatInfo {
	const char *name;
	uint32_t code;
	unsigned int bitsPerPixel;
	PixelFormatInfo::ColourEncoding colourEncoding;
};

/*
 * \var mediaBusFormatInfo
 * \brief A map that associates MediaBusFormatInfo struct to V4L2 media
 * bus codes
 */
const std::map<uint32_t, MediaBusFormatInfo> mediaBusFormatInfo{
	/* This table is sorted to match the order in linux/media-bus-format.h */
	{ MEDIA_BUS_FMT_RGB444_2X8_PADHI_BE, {
		.name = "RGB444_2X8_PADHI_BE",
		.code = MEDIA_BUS_FMT_RGB444_2X8_PADHI_BE,
		.bitsPerPixel = 16,
		.colourEncoding = PixelFormatInfo::ColourEncodingRGB,
	} },
	{ MEDIA_BUS_FMT_RGB444_2X8_PADHI_LE, {
		.name = "RGB444_2X8_PADHI_LE",
		.code = MEDIA_BUS_FMT_RGB444_2X8_PADHI_LE,
		.bitsPerPixel = 16,
		.colourEncoding = PixelFormatInfo::ColourEncodingRGB,
	} },
	{ MEDIA_BUS_FMT_RGB555_2X8_PADHI_BE, {
		.name = "RGB555_2X8_PADHI_BE",
		.code = MEDIA_BUS_FMT_RGB555_2X8_PADHI_BE,
		.bitsPerPixel = 16,
		.colourEncoding = PixelFormatInfo::ColourEncodingRGB,
	} },
	{ MEDIA_BUS_FMT_RGB555_2X8_PADHI_LE, {
		.name = "RGB555_2X8_PADHI_LE",
		.code = MEDIA_BUS_FMT_RGB555_2X8_PADHI_LE,
		.bitsPerPixel = 16,
		.colourEncoding = PixelFormatInfo::ColourEncodingRGB,
	} },
	{ MEDIA_BUS_FMT_RGB565_1X16, {
		.name = "RGB565_1X16",
		.code = MEDIA_BUS_FMT_RGB565_1X16,
		.bitsPerPixel = 16,
		.colourEncoding = PixelFormatInfo::ColourEncodingRGB,
	} },
	{ MEDIA_BUS_FMT_BGR565_2X8_BE, {
		.name = "BGR565_2X8_BE",
		.code = MEDIA_BUS_FMT_BGR565_2X8_BE,
		.bitsPerPixel = 16,
		.colourEncoding = PixelFormatInfo::ColourEncodingRGB,
	} },
	{ MEDIA_BUS_FMT_BGR565_2X8_LE, {
		.name = "BGR565_2X8_LE",
		.code = MEDIA_BUS_FMT_BGR565_2X8_LE,
		.bitsPerPixel = 16,
		.colourEncoding = PixelFormatInfo::ColourEncodingRGB,
	} },
	{ MEDIA_BUS_FMT_RGB565_2X8_BE, {
		.name = "RGB565_2X8_BE",
		.code = MEDIA_BUS_FMT_RGB565_2X8_BE,
		.bitsPerPixel = 16,
		.colourEncoding = PixelFormatInfo::ColourEncodingRGB,
	} },
	{ MEDIA_BUS_FMT_RGB565_2X8_LE, {
		.name = "RGB565_2X8_LE",
		.code = MEDIA_BUS_FMT_RGB565_2X8_LE,
		.bitsPerPixel = 16,
		.colourEncoding = PixelFormatInfo::ColourEncodingRGB,
	} },
	{ MEDIA_BUS_FMT_RGB666_1X18, {
		.name = "RGB666_1X18",
		.code = MEDIA_BUS_FMT_RGB666_1X18,
		.bitsPerPixel = 18,
		.colourEncoding = PixelFormatInfo::ColourEncodingRGB,
	} },
	{ MEDIA_BUS_FMT_BGR888_1X24, {
		.name = "BGR888_1X24",
		.code = MEDIA_BUS_FMT_BGR888_1X24,
		.bitsPerPixel = 24,
		.colourEncoding = PixelFormatInfo::ColourEncodingRGB,
	} },
	{ MEDIA_BUS_FMT_RGB888_1X24, {
		.name = "RGB888_1X24",
		.code = MEDIA_BUS_FMT_RGB888_1X24,
		.bitsPerPixel = 24,
		.colourEncoding = PixelFormatInfo::ColourEncodingRGB,
	} },
	{ MEDIA_BUS_FMT_RGB888_2X12_BE, {
		.name = "RGB888_2X12_BE",
		.code = MEDIA_BUS_FMT_RGB888_2X12_BE,
		.bitsPerPixel = 24,
		.colourEncoding = PixelFormatInfo::ColourEncodingRGB,
	} },
	{ MEDIA_BUS_FMT_RGB888_2X12_LE, {
		.name = "RGB888_2X12_LE",
		.code = MEDIA_BUS_FMT_RGB888_2X12_LE,
		.bitsPerPixel = 24,
		.colourEncoding = PixelFormatInfo::ColourEncodingRGB,
	} },
	{ MEDIA_BUS_FMT_ARGB8888_1X32, {
		.name = "ARGB8888_1X32",
		.code = MEDIA_BUS_FMT_ARGB8888_1X32,
		.bitsPerPixel = 32,
		.colourEncoding = PixelFormatInfo::ColourEncodingRGB,
	} },
	{ MEDIA_BUS_FMT_Y8_1X8, {
		.name = "Y8_1X8",
		.code = MEDIA_BUS_FMT_Y8_1X8,
		.bitsPerPixel = 8,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_UV8_1X8, {
		.name = "UV8_1X8",
		.code = MEDIA_BUS_FMT_UV8_1X8,
		.bitsPerPixel = 8,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_UYVY8_1_5X8, {
		.name = "UYVY8_1_5X8",
		.code = MEDIA_BUS_FMT_UYVY8_1_5X8,
		.bitsPerPixel = 12,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_VYUY8_1_5X8, {
		.name = "VYUY8_1_5X8",
		.code = MEDIA_BUS_FMT_VYUY8_1_5X8,
		.bitsPerPixel = 12,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_YUYV8_1_5X8, {
		.name = "YUYV8_1_5X8",
		.code = MEDIA_BUS_FMT_YUYV8_1_5X8,
		.bitsPerPixel = 12,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_YVYU8_1_5X8, {
		.name = "YVYU8_1_5X8",
		.code = MEDIA_BUS_FMT_YVYU8_1_5X8,
		.bitsPerPixel = 12,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_UYVY8_2X8, {
		.name = "UYVY8_2X8",
		.code = MEDIA_BUS_FMT_UYVY8_2X8,
		.bitsPerPixel = 16,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_VYUY8_2X8, {
		.name = "VYUY8_2X8",
		.code = MEDIA_BUS_FMT_VYUY8_2X8,
		.bitsPerPixel = 16,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_YUYV8_2X8, {
		.name = "YUYV8_2X8",
		.code = MEDIA_BUS_FMT_YUYV8_2X8,
		.bitsPerPixel = 16,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_YVYU8_2X8, {
		.name = "YVYU8_2X8",
		.code = MEDIA_BUS_FMT_YVYU8_2X8,
		.bitsPerPixel = 16,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_Y10_1X10, {
		.name = "Y10_1X10",
		.code = MEDIA_BUS_FMT_Y10_1X10,
		.bitsPerPixel = 10,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_UYVY10_2X10, {
		.name = "UYVY10_2X10",
		.code = MEDIA_BUS_FMT_UYVY10_2X10,
		.bitsPerPixel = 20,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_VYUY10_2X10, {
		.name = "VYUY10_2X10",
		.code = MEDIA_BUS_FMT_VYUY10_2X10,
		.bitsPerPixel = 20,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_YUYV10_2X10, {
		.name = "YUYV10_2X10",
		.code = MEDIA_BUS_FMT_YUYV10_2X10,
		.bitsPerPixel = 20,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_YVYU10_2X10, {
		.name = "YVYU10_2X10",
		.code = MEDIA_BUS_FMT_YVYU10_2X10,
		.bitsPerPixel = 20,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_Y12_1X12, {
		.name = "Y12_1X12",
		.code = MEDIA_BUS_FMT_Y12_1X12,
		.bitsPerPixel = 12,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_Y16_1X16, {
		.name = "Y16_1X16",
		.code = MEDIA_BUS_FMT_Y16_1X16,
		.bitsPerPixel = 16,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_UYVY8_1X16, {
		.name = "UYVY8_1X16",
		.code = MEDIA_BUS_FMT_UYVY8_1X16,
		.bitsPerPixel = 16,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_VYUY8_1X16, {
		.name = "VYUY8_1X16",
		.code = MEDIA_BUS_FMT_VYUY8_1X16,
		.bitsPerPixel = 16,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_YUYV8_1X16, {
		.name = "YUYV8_1X16",
		.code = MEDIA_BUS_FMT_YUYV8_1X16,
		.bitsPerPixel = 16,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_YVYU8_1X16, {
		.name = "YVYU8_1X16",
		.code = MEDIA_BUS_FMT_YVYU8_1X16,
		.bitsPerPixel = 16,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_YDYUYDYV8_1X16, {
		.name = "YDYUYDYV8_1X16",
		.code = MEDIA_BUS_FMT_YDYUYDYV8_1X16,
		.bitsPerPixel = 16,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_UYVY10_1X20, {
		.name = "UYVY10_1X20",
		.code = MEDIA_BUS_FMT_UYVY10_1X20,
		.bitsPerPixel = 20,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_VYUY10_1X20, {
		.name = "VYUY10_1X20",
		.code = MEDIA_BUS_FMT_VYUY10_1X20,
		.bitsPerPixel = 20,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_YUYV10_1X20, {
		.name = "YUYV10_1X20",
		.code = MEDIA_BUS_FMT_YUYV10_1X20,
		.bitsPerPixel = 20,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_YVYU10_1X20, {
		.name = "YVYU10_1X20",
		.code = MEDIA_BUS_FMT_YVYU10_1X20,
		.bitsPerPixel = 20,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_YUV8_1X24, {
		.name = "YUV8_1X24",
		.code = MEDIA_BUS_FMT_YUV8_1X24,
		.bitsPerPixel = 24,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_YUV10_1X30, {
		.name = "YUV10_1X30",
		.code = MEDIA_BUS_FMT_YUV10_1X30,
		.bitsPerPixel = 30,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_AYUV8_1X32, {
		.name = "AYUV8_1X32",
		.code = MEDIA_BUS_FMT_AYUV8_1X32,
		.bitsPerPixel = 32,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_UYVY12_2X12, {
		.name = "UYVY12_2X12",
		.code = MEDIA_BUS_FMT_UYVY12_2X12,
		.bitsPerPixel = 24,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_VYUY12_2X12, {
		.name = "VYUY12_2X12",
		.code = MEDIA_BUS_FMT_VYUY12_2X12,
		.bitsPerPixel = 24,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_YUYV12_2X12, {
		.name = "YUYV12_2X12",
		.code = MEDIA_BUS_FMT_YUYV12_2X12,
		.bitsPerPixel = 24,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_YVYU12_2X12, {
		.name = "YVYU12_2X12",
		.code = MEDIA_BUS_FMT_YVYU12_2X12,
		.bitsPerPixel = 24,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_UYVY12_1X24, {
		.name = "UYVY12_1X24",
		.code = MEDIA_BUS_FMT_UYVY12_1X24,
		.bitsPerPixel = 24,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_VYUY12_1X24, {
		.name = "VYUY12_1X24",
		.code = MEDIA_BUS_FMT_VYUY12_1X24,
		.bitsPerPixel = 24,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_YUYV12_1X24, {
		.name = "YUYV12_1X24",
		.code = MEDIA_BUS_FMT_YUYV12_1X24,
		.bitsPerPixel = 24,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_YVYU12_1X24, {
		.name = "YVYU12_1X24",
		.code = MEDIA_BUS_FMT_YVYU12_1X24,
		.bitsPerPixel = 24,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
	{ MEDIA_BUS_FMT_SBGGR8_1X8, {
		.name = "SBGGR8_1X8",
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.bitsPerPixel = 8,
		.colourEncoding = PixelFormatInfo::ColourEncodingRAW,
	} },
	{ MEDIA_BUS_FMT_SGBRG8_1X8, {
		.name = "SGBRG8_1X8",
		.code = MEDIA_BUS_FMT_SGBRG8_1X8,
		.bitsPerPixel = 8,
		.colourEncoding = PixelFormatInfo::ColourEncodingRAW,
	} },
	{ MEDIA_BUS_FMT_SGRBG8_1X8, {
		.name = "SGRBG8_1X8",
		.code = MEDIA_BUS_FMT_SGRBG8_1X8,
		.bitsPerPixel = 8,
		.colourEncoding = PixelFormatInfo::ColourEncodingRAW,
	} },
	{ MEDIA_BUS_FMT_SRGGB8_1X8, {
		.name = "SRGGB8_1X8",
		.code = MEDIA_BUS_FMT_SRGGB8_1X8,
		.bitsPerPixel = 8,
		.colourEncoding = PixelFormatInfo::ColourEncodingRAW,
	} },
	{ MEDIA_BUS_FMT_SBGGR10_ALAW8_1X8, {
		.name = "SBGGR10_ALAW8_1X8",
		.code = MEDIA_BUS_FMT_SBGGR10_ALAW8_1X8,
		.bitsPerPixel = 8,
		.colourEncoding = PixelFormatInfo::ColourEncodingRAW,
	} },
	{ MEDIA_BUS_FMT_SGBRG10_ALAW8_1X8, {
		.name = "SGBRG10_ALAW8_1X8",
		.code = MEDIA_BUS_FMT_SGBRG10_ALAW8_1X8,
		.bitsPerPixel = 8,
		.colourEncoding = PixelFormatInfo::ColourEncodingRAW,
	} },
	{ MEDIA_BUS_FMT_SGRBG10_ALAW8_1X8, {
		.name = "SGRBG10_ALAW8_1X8",
		.code = MEDIA_BUS_FMT_SGRBG10_ALAW8_1X8,
		.bitsPerPixel = 8,
		.colourEncoding = PixelFormatInfo::ColourEncodingRAW,
	} },
	{ MEDIA_BUS_FMT_SRGGB10_ALAW8_1X8, {
		.name = "SRGGB10_ALAW8_1X8",
		.code = MEDIA_BUS_FMT_SRGGB10_ALAW8_1X8,
		.bitsPerPixel = 8,
		.colourEncoding = PixelFormatInfo::ColourEncodingRAW,
	} },
	{ MEDIA_BUS_FMT_SBGGR10_DPCM8_1X8, {
		.name = "SBGGR10_DPCM8_1X8",
		.code = MEDIA_BUS_FMT_SBGGR10_DPCM8_1X8,
		.bitsPerPixel = 8,
		.colourEncoding = PixelFormatInfo::ColourEncodingRAW,
	} },
	{ MEDIA_BUS_FMT_SGBRG10_DPCM8_1X8, {
		.name = "SGBRG10_DPCM8_1X8",
		.code = MEDIA_BUS_FMT_SGBRG10_DPCM8_1X8,
		.bitsPerPixel = 8,
		.colourEncoding = PixelFormatInfo::ColourEncodingRAW,
	} },
	{ MEDIA_BUS_FMT_SGRBG10_DPCM8_1X8, {
		.name = "SGRBG10_DPCM8_1X8",
		.code = MEDIA_BUS_FMT_SGRBG10_DPCM8_1X8,
		.bitsPerPixel = 8,
		.colourEncoding = PixelFormatInfo::ColourEncodingRAW,
	} },
	{ MEDIA_BUS_FMT_SRGGB10_DPCM8_1X8, {
		.name = "SRGGB10_DPCM8_1X8",
		.code = MEDIA_BUS_FMT_SRGGB10_DPCM8_1X8,
		.bitsPerPixel = 8,
		.colourEncoding = PixelFormatInfo::ColourEncodingRAW,
	} },
	{ MEDIA_BUS_FMT_SBGGR10_2X8_PADHI_BE, {
		.name = "SBGGR10_2X8_PADHI_BE",
		.code = MEDIA_BUS_FMT_SBGGR10_2X8_PADHI_BE,
		.bitsPerPixel = 16,
		.colourEncoding = PixelFormatInfo::ColourEncodingRAW,
	} },
	{ MEDIA_BUS_FMT_SBGGR10_2X8_PADHI_LE, {
		.name = "SBGGR10_2X8_PADHI_LE",
		.code = MEDIA_BUS_FMT_SBGGR10_2X8_PADHI_LE,
		.bitsPerPixel = 16,
		.colourEncoding = PixelFormatInfo::ColourEncodingRAW,
	} },
	{ MEDIA_BUS_FMT_SBGGR10_2X8_PADLO_BE, {
		.name = "SBGGR10_2X8_PADLO_BE",
		.code = MEDIA_BUS_FMT_SBGGR10_2X8_PADLO_BE,
		.bitsPerPixel = 16,
		.colourEncoding = PixelFormatInfo::ColourEncodingRAW,
	} },
	{ MEDIA_BUS_FMT_SBGGR10_2X8_PADLO_LE, {
		.name = "SBGGR10_2X8_PADLO_LE",
		.code = MEDIA_BUS_FMT_SBGGR10_2X8_PADLO_LE,
		.bitsPerPixel = 16,
		.colourEncoding = PixelFormatInfo::ColourEncodingRAW,
	} },
	{ MEDIA_BUS_FMT_SBGGR10_1X10, {
		.name = "SBGGR10_1X10",
		.code = MEDIA_BUS_FMT_SBGGR10_1X10,
		.bitsPerPixel = 10,
		.colourEncoding = PixelFormatInfo::ColourEncodingRAW,
	} },
	{ MEDIA_BUS_FMT_SGBRG10_1X10, {
		.name = "SGBRG10_1X10",
		.code = MEDIA_BUS_FMT_SGBRG10_1X10,
		.bitsPerPixel = 10,
		.colourEncoding = PixelFormatInfo::ColourEncodingRAW,
	} },
	{ MEDIA_BUS_FMT_SGRBG10_1X10, {
		.name = "SGRBG10_1X10",
		.code = MEDIA_BUS_FMT_SGRBG10_1X10,
		.bitsPerPixel = 10,
		.colourEncoding = PixelFormatInfo::ColourEncodingRAW,
	} },
	{ MEDIA_BUS_FMT_SRGGB10_1X10, {
		.name = "SRGGB10_1X10",
		.code = MEDIA_BUS_FMT_SRGGB10_1X10,
		.bitsPerPixel = 10,
		.colourEncoding = PixelFormatInfo::ColourEncodingRAW,
	} },
	{ MEDIA_BUS_FMT_SBGGR12_1X12, {
		.name = "SBGGR12_1X12",
		.code = MEDIA_BUS_FMT_SBGGR12_1X12,
		.bitsPerPixel = 12,
		.colourEncoding = PixelFormatInfo::ColourEncodingRAW,
	} },
	{ MEDIA_BUS_FMT_SGBRG12_1X12, {
		.name = "SGBRG12_1X12",
		.code = MEDIA_BUS_FMT_SGBRG12_1X12,
		.bitsPerPixel = 12,
		.colourEncoding = PixelFormatInfo::ColourEncodingRAW,
	} },
	{ MEDIA_BUS_FMT_SGRBG12_1X12, {
		.name = "SGRBG12_1X12",
		.code = MEDIA_BUS_FMT_SGRBG12_1X12,
		.bitsPerPixel = 12,
		.colourEncoding = PixelFormatInfo::ColourEncodingRAW,
	} },
	{ MEDIA_BUS_FMT_SRGGB12_1X12, {
		.name = "SRGGB12_1X12",
		.code = MEDIA_BUS_FMT_SRGGB12_1X12,
		.bitsPerPixel = 12,
		.colourEncoding = PixelFormatInfo::ColourEncodingRAW,
	} },
	/* \todo Clarify colour encoding for HSV formats */
	{ MEDIA_BUS_FMT_AHSV8888_1X32, {
		.name = "AHSV8888_1X32",
		.code = MEDIA_BUS_FMT_AHSV8888_1X32,
		.bitsPerPixel = 32,
		.colourEncoding = PixelFormatInfo::ColourEncodingRGB,
	} },
	{ MEDIA_BUS_FMT_JPEG_1X8, {
		.name = "JPEG_1X8",
		.code = MEDIA_BUS_FMT_JPEG_1X8,
		.bitsPerPixel = 8,
		.colourEncoding = PixelFormatInfo::ColourEncodingYUV,
	} },
};

} /* namespace */

/**
 * \struct V4L2SubdeviceCapability
 * \brief struct v4l2_subdev_capability object wrapper and helpers
 *
 * The V4L2SubdeviceCapability structure manages the information returned by the
 * VIDIOC_SUBDEV_QUERYCAP ioctl.
 */

/**
 * \fn V4L2SubdeviceCapability::isReadOnly()
 * \brief Retrieve if a subdevice is registered as read-only
 *
 * A V4L2 subdevice is registered as read-only if V4L2_SUBDEV_CAP_RO_SUBDEV
 * is listed as part of its capabilities.
 *
 * \return True if the subdevice is registered as read-only, false otherwise
 */

/**
 * \fn V4L2SubdeviceCapability::hasStreams()
 * \brief Retrieve if a subdevice supports the V4L2 streams API
 * \return True if the subdevice supports the streams API, false otherwise
 */

/**
 * \struct V4L2SubdeviceFormat
 * \brief The V4L2 sub-device image format and sizes
 *
 * This structure describes the format of images when transported between
 * separate components connected through a physical bus, such as image sensor
 * and image receiver or between components part of the same System-on-Chip that
 * realize an image transformation pipeline.
 *
 * The format of images when transported on physical interconnections is known
 * as the "media bus format", and it is identified by a resolution and a pixel
 * format identification code, known as the "media bus code", not to be confused
 * with the fourcc code that identify the format of images when stored in memory
 * (see V4L2VideoDevice::V4L2DeviceFormat).
 *
 * Media Bus formats supported by the V4L2 APIs are described in Section
 * 4.15.3.4.1 of the "Part I - Video for Linux API" chapter of the "Linux Media
 * Infrastructure userspace API", part of the Linux kernel documentation.
 *
 * Image media bus formats are properties of the subdev pads.  When images are
 * transported between two media pads identified by a 0-indexed number, the
 * image bus format configured on the two pads should match (according to the
 * underlying driver format matching criteria) in order to prepare for a
 * successful streaming operation. For a more detailed description of the image
 * format negotiation process when performed between V4L2 subdevices, refer to
 * Section 4.15.3.1 of the above mentioned Linux kernel documentation section.
 */

/**
 * \var V4L2SubdeviceFormat::mbus_code
 * \brief The image format bus code
 */

/**
 * \var V4L2SubdeviceFormat::size
 * \brief The image size in pixels
 */

/**
 * \var V4L2SubdeviceFormat::colorSpace
 * \brief The color space of the pixels
 *
 * The color space of the image. When setting the format this may be
 * unset, in which case the driver gets to use its default color space.
 * After being set, this value should contain the color space that
 * was actually used. If this value is unset, then the color space chosen
 * by the driver could not be represented by the ColorSpace class (and
 * should probably be added).
 *
 * It is up to the pipeline handler or application to check if the
 * resulting color space is acceptable.
 */

/**
 * \brief Assemble and return a string describing the format
 * \return A string describing the V4L2SubdeviceFormat
 */
const std::string V4L2SubdeviceFormat::toString() const
{
	std::stringstream ss;
	ss << *this;

	return ss.str();
}

/**
 * \brief Retrieve the number of bits per pixel for the V4L2 subdevice format
 * \return The number of bits per pixel for the format, or 0 if the format is
 * not supported
 */
uint8_t V4L2SubdeviceFormat::bitsPerPixel() const
{
	const auto it = mediaBusFormatInfo.find(mbus_code);
	if (it == mediaBusFormatInfo.end()) {
		LOG(V4L2, Error) << "No information available for format '"
				 << *this << "'";
		return 0;
	}

	return it->second.bitsPerPixel;
}

/**
 * \brief Insert a text representation of a V4L2SubdeviceFormat into an output
 * stream
 * \param[in] out The output stream
 * \param[in] f The V4L2SubdeviceFormat
 * \return The output stream \a out
 */
std::ostream &operator<<(std::ostream &out, const V4L2SubdeviceFormat &f)
{
	out << f.size << "-";

	const auto it = mediaBusFormatInfo.find(f.mbus_code);

	if (it == mediaBusFormatInfo.end())
		out << utils::hex(f.mbus_code, 4);
	else
		out << it->second.name;

	return out;
}

/**
 * \class V4L2Subdevice
 * \brief A V4L2 subdevice as exposed by the Linux kernel
 *
 * The V4L2Subdevice class provides an API to the "Sub-device interface" as
 * described in section 4.15 of the "Linux Media Infrastructure userspace API"
 * chapter of the Linux Kernel documentation.
 *
 * A V4L2Subdevice is constructed from a MediaEntity instance, using the system
 * path of the entity's device node. No API call other than open(), isOpen()
 * and close() shall be called on an unopened device instance. Upon destruction
 * any device left open will be closed, and any resources released.
 */

/**
 * \typedef V4L2Subdevice::Formats
 * \brief A map of supported media bus formats to frame sizes
 */

/**
 * \enum V4L2Subdevice::Whence
 * \brief Specify the type of format for getFormat() and setFormat() operations
 * \var V4L2Subdevice::ActiveFormat
 * \brief The format operation applies to ACTIVE formats
 * \var V4L2Subdevice::TryFormat
 * \brief The format operation applies to TRY formats
 */

/**
 * \class V4L2Subdevice::Routing
 * \brief V4L2 subdevice routing table
 *
 * This class stores a subdevice routing table as a vector of routes.
 */

/**
 * \brief Assemble and return a string describing the routing table
 * \return A string describing the routing table
 */
std::string V4L2Subdevice::Routing::toString() const
{
	std::stringstream routing;

	for (const auto &[i, route] : utils::enumerate(*this)) {
		routing << "[" << i << "] "
			<< route.sink_pad << "/" << route.sink_stream << " -> "
			<< route.source_pad << "/" << route.source_stream
			<< " (" << utils::hex(route.flags) << ")";
		if (i != size() - 1)
			routing << ", ";
	}

	return routing.str();
}

/**
 * \brief Create a V4L2 subdevice from a MediaEntity using its device node
 * path
 */
V4L2Subdevice::V4L2Subdevice(const MediaEntity *entity)
	: V4L2Device(entity->deviceNode()), entity_(entity)
{
}

V4L2Subdevice::~V4L2Subdevice()
{
	close();
}

/**
 * \brief Open a V4L2 subdevice
 * \return 0 on success or a negative error code otherwise
 */
int V4L2Subdevice::open()
{
	int ret = V4L2Device::open(O_RDWR);
	if (ret)
		return ret;

	/*
	 * Try to query the subdev capabilities. The VIDIOC_SUBDEV_QUERYCAP API
	 * was introduced in kernel v5.8, ENOTTY errors must be ignored to
	 * support older kernels.
	 */
	caps_ = {};
	ret = ioctl(VIDIOC_SUBDEV_QUERYCAP, &caps_);
	if (ret < 0 && errno != ENOTTY) {
		ret = -errno;
		LOG(V4L2, Error)
			<< "Unable to query capabilities: " << strerror(-ret);
		return ret;
	}

	/* If the subdev supports streams, enable the streams API. */
	if (caps_.hasStreams()) {
		struct v4l2_subdev_client_capability clientCaps{};
		clientCaps.capabilities = V4L2_SUBDEV_CLIENT_CAP_STREAMS;

		ret = ioctl(VIDIOC_SUBDEV_S_CLIENT_CAP, &clientCaps);
		if (ret < 0) {
			ret = -errno;
			LOG(V4L2, Error)
				<< "Unable to set client capabilities: "
				<< strerror(-ret);
			return ret;
		}
	}

	return 0;
}

/**
 * \fn V4L2Subdevice::entity()
 * \brief Retrieve the media entity associated with the subdevice
 * \return The subdevice's associated media entity.
 */

/**
 * \brief Get selection rectangle \a rect for \a target
 * \param[in] pad The 0-indexed pad number the rectangle is retrieved from
 * \param[in] target The selection target defined by the V4L2_SEL_TGT_* flags
 * \param[out] rect The retrieved selection rectangle
 *
 * \todo Define a V4L2SelectionTarget enum for the selection target
 *
 * \return 0 on success or a negative error code otherwise
 */
int V4L2Subdevice::getSelection(unsigned int pad, unsigned int target,
				Rectangle *rect)
{
	struct v4l2_subdev_selection sel = {};

	sel.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	sel.pad = pad;
	sel.target = target;
	sel.flags = 0;

	int ret = ioctl(VIDIOC_SUBDEV_G_SELECTION, &sel);
	if (ret < 0) {
		LOG(V4L2, Error)
			<< "Unable to get rectangle " << target << " on pad "
			<< pad << ": " << strerror(-ret);
		return ret;
	}

	rect->x = sel.r.left;
	rect->y = sel.r.top;
	rect->width = sel.r.width;
	rect->height = sel.r.height;

	return 0;
}

/**
 * \brief Set selection rectangle \a rect for \a target
 * \param[in] pad The 0-indexed pad number the rectangle is to be applied to
 * \param[in] target The selection target defined by the V4L2_SEL_TGT_* flags
 * \param[inout] rect The selection rectangle to be applied
 *
 * \todo Define a V4L2SelectionTarget enum for the selection target
 *
 * \return 0 on success or a negative error code otherwise
 */
int V4L2Subdevice::setSelection(unsigned int pad, unsigned int target,
				Rectangle *rect)
{
	struct v4l2_subdev_selection sel = {};

	sel.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	sel.pad = pad;
	sel.target = target;
	sel.flags = 0;

	sel.r.left = rect->x;
	sel.r.top = rect->y;
	sel.r.width = rect->width;
	sel.r.height = rect->height;

	int ret = ioctl(VIDIOC_SUBDEV_S_SELECTION, &sel);
	if (ret < 0) {
		LOG(V4L2, Error)
			<< "Unable to set rectangle " << target << " on pad "
			<< pad << ": " << strerror(-ret);
		return ret;
	}

	rect->x = sel.r.left;
	rect->y = sel.r.top;
	rect->width = sel.r.width;
	rect->height = sel.r.height;

	return 0;
}
/**
 * \brief Enumerate all media bus codes and frame sizes on a \a pad
 * \param[in] pad The 0-indexed pad number to enumerate formats on
 *
 * Enumerate all media bus codes and frame sizes supported by the subdevice on
 * a \a pad.
 *
 * \return A list of the supported device formats
 */
V4L2Subdevice::Formats V4L2Subdevice::formats(unsigned int pad)
{
	Formats formats;

	if (pad >= entity_->pads().size()) {
		LOG(V4L2, Error) << "Invalid pad: " << pad;
		return {};
	}

	for (unsigned int code : enumPadCodes(pad)) {
		std::vector<SizeRange> sizes = enumPadSizes(pad, code);
		if (sizes.empty())
			return {};

		const auto inserted = formats.insert({ code, sizes });
		if (!inserted.second) {
			LOG(V4L2, Error)
				<< "Could not add sizes for media bus code "
				<< code << " on pad " << pad;
			return {};
		}
	}

	return formats;
}

std::optional<ColorSpace> V4L2Subdevice::toColorSpace(const v4l2_mbus_framefmt &format) const
{
	/*
	 * Only image formats have a color space, for other formats (such as
	 * metadata formats) the color space concept isn't applicable. V4L2
	 * subdev drivers return a colorspace set to V4L2_COLORSPACE_DEFAULT in
	 * that case (as well as for image formats when the driver hasn't
	 * bothered implementing color space support). Check the colorspace
	 * field here and return std::nullopt directly to avoid logging a
	 * warning.
	 */
	if (format.colorspace == V4L2_COLORSPACE_DEFAULT)
		return std::nullopt;

	PixelFormatInfo::ColourEncoding colourEncoding;
	auto iter = mediaBusFormatInfo.find(format.code);
	if (iter != mediaBusFormatInfo.end()) {
		colourEncoding = iter->second.colourEncoding;
	} else {
		LOG(V4L2, Warning)
			<< "Unknown subdev format "
			<< utils::hex(format.code, 4)
			<< ", defaulting to RGB encoding";

		colourEncoding = PixelFormatInfo::ColourEncodingRGB;
	}

	return V4L2Device::toColorSpace(format, colourEncoding);
}

/**
 * \brief Retrieve the image format set on one of the V4L2 subdevice pads
 * \param[in] pad The 0-indexed pad number the format is to be retrieved from
 * \param[out] format The image bus format
 * \param[in] whence The format to get, \ref V4L2Subdevice::ActiveFormat
 * "ActiveFormat" or \ref V4L2Subdevice::TryFormat "TryFormat"
 * \return 0 on success or a negative error code otherwise
 */
int V4L2Subdevice::getFormat(unsigned int pad, V4L2SubdeviceFormat *format,
			     Whence whence)
{
	struct v4l2_subdev_format subdevFmt = {};
	subdevFmt.which = whence;
	subdevFmt.pad = pad;

	int ret = ioctl(VIDIOC_SUBDEV_G_FMT, &subdevFmt);
	if (ret) {
		LOG(V4L2, Error)
			<< "Unable to get format on pad " << pad
			<< ": " << strerror(-ret);
		return ret;
	}

	format->size.width = subdevFmt.format.width;
	format->size.height = subdevFmt.format.height;
	format->mbus_code = subdevFmt.format.code;
	format->colorSpace = toColorSpace(subdevFmt.format);

	return 0;
}

/**
 * \brief Set an image format on one of the V4L2 subdevice pads
 * \param[in] pad The 0-indexed pad number the format is to be applied to
 * \param[inout] format The image bus format to apply to the subdevice's pad
 * \param[in] whence The format to set, \ref V4L2Subdevice::ActiveFormat
 * "ActiveFormat" or \ref V4L2Subdevice::TryFormat "TryFormat"
 *
 * Apply the requested image format to the desired media pad and return the
 * actually applied format parameters, as getFormat() would do.
 *
 * \return 0 on success or a negative error code otherwise
 */
int V4L2Subdevice::setFormat(unsigned int pad, V4L2SubdeviceFormat *format,
			     Whence whence)
{
	struct v4l2_subdev_format subdevFmt = {};
	subdevFmt.which = whence;
	subdevFmt.pad = pad;
	subdevFmt.format.width = format->size.width;
	subdevFmt.format.height = format->size.height;
	subdevFmt.format.code = format->mbus_code;
	subdevFmt.format.field = V4L2_FIELD_NONE;
	if (format->colorSpace) {
		fromColorSpace(format->colorSpace, subdevFmt.format);

		/* The CSC flag is only applicable to source pads. */
		if (entity_->pads()[pad]->flags() & MEDIA_PAD_FL_SOURCE)
			subdevFmt.format.flags |= V4L2_MBUS_FRAMEFMT_SET_CSC;
	}

	int ret = ioctl(VIDIOC_SUBDEV_S_FMT, &subdevFmt);
	if (ret) {
		LOG(V4L2, Error)
			<< "Unable to set format on pad " << pad
			<< ": " << strerror(-ret);
		return ret;
	}

	format->size.width = subdevFmt.format.width;
	format->size.height = subdevFmt.format.height;
	format->mbus_code = subdevFmt.format.code;
	format->colorSpace = toColorSpace(subdevFmt.format);

	return 0;
}

/**
 * \brief Retrieve the subdevice's internal routing table
 * \param[out] routing The routing table
 * \param[in] whence The routing table to get, \ref V4L2Subdevice::ActiveFormat
 * "ActiveFormat" or \ref V4L2Subdevice::TryFormat "TryFormat"
 *
 * \return 0 on success or a negative error code otherwise
 */
int V4L2Subdevice::getRouting(Routing *routing, Whence whence)
{
	if (!caps_.hasStreams())
		return 0;

	struct v4l2_subdev_routing rt = {};

	rt.which = whence;

	int ret = ioctl(VIDIOC_SUBDEV_G_ROUTING, &rt);
	if (ret == 0 || ret == -ENOTTY)
		return ret;

	if (ret != -ENOSPC) {
		LOG(V4L2, Error)
			<< "Failed to retrieve number of routes: "
			<< strerror(-ret);
		return ret;
	}

	routing->resize(rt.num_routes);
	rt.routes = reinterpret_cast<uintptr_t>(routing->data());

	ret = ioctl(VIDIOC_SUBDEV_G_ROUTING, &rt);
	if (ret) {
		LOG(V4L2, Error)
			<< "Failed to retrieve routes: " << strerror(-ret);
		return ret;
	}

	if (rt.num_routes != routing->size()) {
		LOG(V4L2, Error) << "Invalid number of routes";
		return -EINVAL;
	}

	return 0;
}

/**
 * \brief Set a routing table on the V4L2 subdevice
 * \param[inout] routing The routing table
 * \param[in] whence The routing table to set, \ref V4L2Subdevice::ActiveFormat
 * "ActiveFormat" or \ref V4L2Subdevice::TryFormat "TryFormat"
 *
 * Apply to the V4L2 subdevice the routing table \a routing and update its
 * content to reflect the actually applied routing table as getRouting() would
 * do.
 *
 * \return 0 on success or a negative error code otherwise
 */
int V4L2Subdevice::setRouting(Routing *routing, Whence whence)
{
	if (!caps_.hasStreams())
		return 0;

	struct v4l2_subdev_routing rt = {};
	rt.which = whence;
	rt.num_routes = routing->size();
	rt.routes = reinterpret_cast<uintptr_t>(routing->data());

	int ret = ioctl(VIDIOC_SUBDEV_S_ROUTING, &rt);
	if (ret) {
		LOG(V4L2, Error) << "Failed to set routes: " << strerror(-ret);
		return ret;
	}

	routing->resize(rt.num_routes);

	return 0;
}

/**
 * \brief Retrieve the model name of the device
 *
 * The model name allows identification of the specific device model. This can
 * be used to infer device characteristics, for instance to determine the
 * analogue gain model of a camera sensor based on the sensor model name.
 *
 * Neither the V4L2 API nor the Media Controller API expose an explicit model
 * name. This function implements a heuristics to extract the model name from
 * the subdevice's entity name. This should produce accurate results for
 * I2C-based devices. If the heuristics can't match a known naming pattern,
 * the function returns the full entity name.
 *
 * \return The model name of the device
 */
const std::string &V4L2Subdevice::model()
{
	if (!model_.empty())
		return model_;

	/*
	 * Extract model name from the media entity name.
	 *
	 * There is no standardized naming scheme for sensor or other entities
	 * in the Linux kernel at the moment.
	 *
	 * - The most common rule, used by I2C sensors, associates the model
	 *   name with the I2C bus number and address (e.g. 'imx219 0-0010').
	 *
	 * - When the sensor exposes multiple subdevs, the model name is
	 *   usually followed by a function name, as in the smiapp driver (e.g.
	 *   'jt8ew9 pixel_array 0-0010').
	 *
	 * - The vimc driver names its sensors 'Sensor A' and 'Sensor B'.
	 *
	 * Other schemes probably exist. As a best effort heuristic, use the
	 * part of the entity name before the first space if the name contains
	 * an I2C address, and use the full entity name otherwise.
	 */
	std::string entityName = entity_->name();
	std::regex i2cRegex{ " [0-9]+-[0-9a-f]{4}" };
	std::smatch match;

	std::string model;
	if (std::regex_search(entityName, match, i2cRegex))
		model_ = entityName.substr(0, entityName.find(' '));
	else
		model_ = entityName;

	return model_;
}

/**
 * \fn V4L2Subdevice::caps()
 * \brief Retrieve the subdevice V4L2 capabilities
 * \return The subdevice V4L2 capabilities
 */

/**
 * \brief Create a new video subdevice instance from \a entity in media device
 * \a media
 * \param[in] media The media device where the entity is registered
 * \param[in] entity The media entity name
 *
 * \return A newly created V4L2Subdevice on success, nullptr otherwise
 */
std::unique_ptr<V4L2Subdevice>
V4L2Subdevice::fromEntityName(const MediaDevice *media,
			      const std::string &entity)
{
	MediaEntity *mediaEntity = media->getEntityByName(entity);
	if (!mediaEntity)
		return nullptr;

	return std::make_unique<V4L2Subdevice>(mediaEntity);
}

std::string V4L2Subdevice::logPrefix() const
{
	return "'" + entity_->name() + "'";
}

std::vector<unsigned int> V4L2Subdevice::enumPadCodes(unsigned int pad)
{
	std::vector<unsigned int> codes;
	int ret;

	for (unsigned int index = 0; ; index++) {
		struct v4l2_subdev_mbus_code_enum mbusEnum = {};
		mbusEnum.pad = pad;
		mbusEnum.index = index;
		mbusEnum.which = V4L2_SUBDEV_FORMAT_ACTIVE;

		ret = ioctl(VIDIOC_SUBDEV_ENUM_MBUS_CODE, &mbusEnum);
		if (ret)
			break;

		codes.push_back(mbusEnum.code);
	}

	if (ret < 0 && ret != -EINVAL) {
		LOG(V4L2, Error)
			<< "Unable to enumerate formats on pad " << pad
			<< ": " << strerror(-ret);
		return {};
	}

	return codes;
}

std::vector<SizeRange> V4L2Subdevice::enumPadSizes(unsigned int pad,
						   unsigned int code)
{
	std::vector<SizeRange> sizes;
	int ret;

	for (unsigned int index = 0;; index++) {
		struct v4l2_subdev_frame_size_enum sizeEnum = {};
		sizeEnum.index = index;
		sizeEnum.pad = pad;
		sizeEnum.code = code;
		sizeEnum.which = V4L2_SUBDEV_FORMAT_ACTIVE;

		ret = ioctl(VIDIOC_SUBDEV_ENUM_FRAME_SIZE, &sizeEnum);
		if (ret)
			break;

		sizes.emplace_back(Size{ sizeEnum.min_width, sizeEnum.min_height },
				   Size{ sizeEnum.max_width, sizeEnum.max_height });
	}

	if (ret < 0 && ret != -EINVAL && ret != -ENOTTY) {
		LOG(V4L2, Error)
			<< "Unable to enumerate sizes on pad " << pad
			<< ": " << strerror(-ret);
		return {};
	}

	return sizes;
}

} /* namespace libcamera */
