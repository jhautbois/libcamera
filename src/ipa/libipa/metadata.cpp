/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Based on the implementation from the Raspberry Pi IPA,
 * Copyright (C) 2019-2021, Raspberry Pi (Trading) Ltd.
 * Copyright (C) 2021, Ideas On Board
 *
 * metadata.cpp -  libipa metadata class
 */

#include "metadata.h"

/**
 * \file metadata.h
 * \brief A metadata class to share objects
 */

namespace libcamera {

namespace ipa {

/**
 * \class Metadata
 * \brief A simple class for carrying arbitrary metadata, for example
 * about an image. It is used to exchange data between algorithms.
 */

/**
 * \param[in] other A Metadata object
 *
 * Stores the data from one Metadata to another one
 */
Metadata::Metadata(Metadata const &other)
{
	std::scoped_lock other_lock(other.mutex_);
	data_ = other.data_;
}

/**
 * \param[in] tag A string used as the key in a map
 * \param[in] value The value to set into the map
 *
 * Sets the value in the map to the tag key. The mutex is
 * taken for the duration of the block.
 */
template<typename T>
void Metadata::set(std::string const &tag, T const &value)
{
	std::scoped_lock lock(mutex_);
	data_[tag] = value;
}

/**
 * \param[in] tag A string used as the key in a map
 * \param[in] value The value to set into the map
 *
 * Gets the value in the map of the tag key. The mutex is
 * taken for the duration of the block.
 *
 * \return 0 if value is found, -1 if not existent
 */
template<typename T>
int Metadata::get(std::string const &tag, T &value) const
{
	std::scoped_lock lock(mutex_);
	auto it = data_.find(tag);
	if (it == data_.end())
		return -1;
	value = std::any_cast<T>(it->second);
	return 0;
}

/**
 * Clear the Metadata map. The mutex is taken for the duration of
 * the block.
 */
void Metadata::clear()
{
	std::scoped_lock lock(mutex_);
	data_.clear();
}

/**
 * \param[in] other A metadata to merge with
 * Merge two Metadata maps. The mutex is taken for the duration of
 * the block.
 */
void Metadata::merge(Metadata &other)
{
	std::scoped_lock lock(mutex_, other.mutex_);
	data_.merge(other.data_);
}

/**
 * \param[in] tag A string used as the key in a map
 *
 * Set the value of the tag key in the map.
 * This allows in-place access to the Metadata contents,
 * for which you should be holding the lock.
 */
template<typename T>
T *Metadata::getLocked(std::string const &tag)
{
	auto it = data_.find(tag);
	if (it == data_.end())
		return nullptr;
	return std::any_cast<T>(&it->second);
}

/**
 * \param[in] tag A string used as the key in a map
 * \param[in] value The value to set into the map
 *
 * Set the value to the tag key in the map.
 * This allows in-place access to the Metadata contents,
 * for which you should be holding the lock.
 */
template<typename T>
void Metadata::setLocked(std::string const &tag, T const &value)
{
	data_[tag] = value;
}

/**
 * Lock the mutex with the standard classes.
 * e.g. std::lock_guard<RPiController::Metadata> lock(metadata)
 */
void Metadata::lock()
{
	mutex_.lock();
}
/**
 * Unlock the mutex with the standard classes.
 * e.g. std::lock_guard<RPiController::Metadata> lock(metadata)
 */
void Metadata::unlock()
{
	mutex_.unlock();
}

} /* namespace ipa */

} /* namespace libcamera */

