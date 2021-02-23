/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * histogram.h - histogram calculation interface
 */
#ifndef __LIBCAMERA_IPA_LIBIPA_HISTOGRAM_H__
#define __LIBCAMERA_IPA_LIBIPA_HISTOGRAM_H__

#include <stdint.h>
#include <vector>
#include <cassert>

// A simple histogram class, for use in particular to find "quantiles" and
// averages between "quantiles".

namespace libcamera {

namespace ipa {

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

	uint32_t bins() const { return cumulative_.size() - 1; }
	uint64_t total() const { return cumulative_[cumulative_.size() - 1]; }
	uint64_t cumulativeFreq(double bin) const;
	double quantile(double q, int first = -1, int last = -1) const;
	double interQuantileMean(double lowQuantile, double hiQuantile) const;

private:
	std::vector<uint64_t> cumulative_;
};

} /* namespace ipa */

} /* namespace libcamera */

#endif /* __LIBCAMERA_IPA_LIBIPA_HISTOGRAM_H__ */