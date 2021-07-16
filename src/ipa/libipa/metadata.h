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

template<typename T>
struct Tag {
	const char *tag_;
};

class Metadata
{
public:
	Metadata() = default;
	~Metadata() = default;

	Metadata(Metadata const &other)
	{
		std::scoped_lock other_lock(other.mutex_);
		data_ = other.data_;
	}

	Metadata(Metadata &&other)
	{
		std::scoped_lock other_lock(other.mutex_);
		data_ = std::move(other.data_);
		other.data_.clear();
	}

	Metadata &operator=(Metadata const &other) = delete;
	Metadata &operator=(Metadata &&other) = delete;

	template<typename T>
	void set(const Tag<T> &tag, T const &value)
	{
		std::scoped_lock lock(mutex_);
		data_[tag.tag_] = value;
	}

	template<typename T>
	int get(const Tag<T> &tag, T &value) const
	{
		std::scoped_lock lock(mutex_);
		auto it = data_.find(tag.tag_);
		if (it == data_.end())
			return -1;
		value = std::any_cast<T>(it->second);
		return 0;
	}

	void clear()
	{
		std::scoped_lock lock(mutex_);
		data_.clear();
	}

	void merge(Metadata &other)
	{
		std::scoped_lock lock(mutex_, other.mutex_);
		data_.merge(other.data_);
	}

	template<typename T>
	T *getLocked(const Tag<T> &tag)
	{
		auto it = data_.find(tag.tag_);
		if (it == data_.end())
			return nullptr;
		return std::any_cast<T>(&it->second);
	}

	template<typename T>
	void setLocked(const Tag<T> &tag, T const &value)
	{
		data_[tag.tag_] = value;
	}

	void lock() { mutex_.lock(); }
	void unlock() { mutex_.unlock(); }

private:
	mutable std::mutex mutex_;
	std::map<const char *, std::any> data_;
};

} /* namespace ipa */

} /* namespace libcamera */

#endif /* __LIBCAMERA_IPA_LIBIPA_METADATA_H__ */
