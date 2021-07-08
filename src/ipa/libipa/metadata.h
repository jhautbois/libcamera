/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Based on the implementation from the Raspberry Pi IPA,
 * Copyright (C) 2019-2021, Raspberry Pi (Trading) Ltd.
 * Copyright (C) 2021, Ideas On Board
 *
 * metadata.h - libipa metadata class
 */
#ifndef __LIBCAMERA_IPA_LIBIPA_METADATA_H__
#define __LIBCAMERA_IPA_LIBIPA_METADATA_H__

#include <any>
#include <map>
#include <memory>
#include <mutex>
#include <string>

namespace libcamera {

namespace ipa {

class Metadata
{
public:
	Metadata() = default;
	Metadata(Metadata const &other);

	template<typename T>
	void set(std::string const &tag, T const &value);

	template<typename T>
	int get(std::string const &tag, T &value) const;

	void clear();
	void merge(Metadata &other);

	template<typename T>
	T *getLocked(std::string const &tag);

	template<typename T>
	void setLocked(std::string const &tag, T const &value);

	void lock();
	void unlock();

private:
	mutable std::mutex mutex_;
	std::map<std::string, std::any> data_;
};

} /* namespace ipa */

} /* namespace libcamera */

#endif /* __LIBCAMERA_IPA_LIBIPA_METADATA_H__ */
