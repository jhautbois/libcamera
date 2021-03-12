/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * histogram.hpp - histogram calculation interface
 */
#pragma once

#include <stdint.h>
#include <vector>
#include <cassert>

namespace RPiController {

class Histogram
{
public:
	template<typename T> Histogram(T *histogram, int num)
	{
		assert(num);
		cumulative_.reserve(num + 1);
		cumulative_.push_back(0);
		for (int i = 0; i < num; i++)
			cumulative_.push_back(cumulative_.back() +
					      histogram[i]);
	}
	uint32_t Bins() const { return cumulative_.size() - 1; }
	uint64_t Total() const { return cumulative_[cumulative_.size() - 1]; }
	uint64_t CumulativeFreq(double bin) const;
	Histogram::quantile(double q, uint32_t first = 0, uint32_t last = UINT_MAX) const;
	double InterQuantileMean(double q_lo, double q_hi) const;

private:
	std::vector<uint64_t> cumulative_;
};

} // namespace RPiController
