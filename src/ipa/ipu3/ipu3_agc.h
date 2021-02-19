/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * ipu3_agc.h - AGC/AEC control algorithm
 */
#ifndef __LIBCAMERA_IPU3_AGC_H__
#define __LIBCAMERA_IPU3_AGC_H__

#include <mutex>
#include <vector>

#include <linux/intel-ipu3.h>

#include <libcamera/geometry.h>

#include <libcamera/ipa/agc_algorithm.h>
#include <libcamera/ipa/ipa_controller.h>

#define AGC_STATS_SIZE 15

namespace libcamera {

struct AgcMeteringMode {
	double weights[AGC_STATS_SIZE];
	Rectangle metering_region[AGC_STATS_SIZE];
};

struct AgcExposureMode {
	std::vector<double> shutter;
	std::vector<double> gain;
};

struct AgcConfig {
	std::map<std::string, AgcMeteringMode> metering_modes;
	std::map<std::string, AgcExposureMode> exposure_modes;
	double speed;
	uint16_t startup_frames;
	unsigned int convergence_frames;
	std::string default_metering_mode;
	std::string default_exposure_mode;
	double default_exposure_time;
	double default_analogue_gain;
};

class IPU3Agc : public AgcAlgorithm
{
public:
	IPU3Agc(IPAController *IPAcontroller);
	char const *Name() const override;
	unsigned int GetConvergenceFrames() const override;
	void SetEv(double ev) override;
	void SetFlickerPeriod(double flicker_period) override;
	void SetMaxShutter(double max_shutter) override; // microseconds
	void SetFixedShutter(double fixed_shutter) override; // microseconds
	void SetFixedAnalogueGain(double fixed_analogue_gain) override;
	void SetMeteringMode(std::string const &metering_mode_name) override;
	void SetExposureMode(std::string const &exposure_mode_name) override;
	void Prepare() override;
	void Process() override;
	void Process(const ipu3_uapi_stats_3a *stats, uint32_t &exposure, uint32_t &gain);
	bool Converged();

private:
	void moments(std::unordered_map<uint32_t, uint32_t> &data, int n);
	void processBrightness(const ipu3_uapi_stats_3a *stats);
	void lockExposure(uint32_t &exposure, uint32_t &gain);

	AgcConfig config_;
	std::string metering_mode_name_;
	std::string exposure_mode_name_;
	uint64_t frameCount_;
	uint64_t lastFrame_;

	/* Vector of calculated brightness for each cell */
	std::vector<uint32_t> brightnessVec_;
	double ev_;
	double flicker_period_;
	double max_shutter_;
	double fixed_shutter_;
	double fixed_analogue_gain_;

	/* Values for filtering */
	uint32_t prevExposure_;
	uint32_t currentExposure_;
	uint32_t nextExposure_;

	double skew_;
	double prevSkew_;
	double currentSkew_;
	bool algoConverged_;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_IPU3_AGC_H__ */
