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
 * \brief A metadata class to provide key based access to arbitrary metadata
 * types.
 */

namespace libcamera {

namespace ipa {

/**
 * \class Metadata
 * \brief A simple class for carrying arbitrary metadata, for example
 * about an image. It is used to exchange data between algorithms.
 *
 * Data is stored as a map with a string based key.
 * The metadata is stored through a std::any() type which is definable by
 * the user, and must be correctly known by both the producer and consumer.
 *
 * Accessing the metadata with an incorrect type will cause undefined
 * behaviour.
 */

/**
 * \fn Metadata::Metadata(Metadata const &other)
 * \param[in] other A Metadata object
 *
 * Copy the data from the \a other Metadata object to this one.
 */

/**
 * \fn Metadata::Metadata(Metadata &&other)
 * \param[in] other A Metadata object
 *
 * Move the data from the \a other Metadata referenced object to this one.
 */

/**
 * \fn Metadata::set(std::string const &tag, T const &value)
 * \param[in] tag A string used as the key
 * \param[in] value The value to set
 *
 * Sets the value in the map to the tag key. This function locks the metadata
 * to protect from concurrent access
 */

/**
 * \fn Metadata::get(std::string const &tag, T &value)
 * \param[in] tag A string used as the key in a map
 * \param[in] value The value to set into the map
 *
 * Gets the value in the map of the tag key. The mutex is
 * taken for the duration of the block.
 *
 * \return 0 if value is found, -1 if not existent
 */

/**
 * \fn Metadata::clear()
 * Clear the Metadata map. This function locks the metadata
 * to protect from concurrent access
 */

/**
 * \fn Metadata::merge(Metadata &other)
 * \param[in] other A metadata to merge with
 * Merge two Metadata maps. This function locks the metadata
 * to protect from concurrent access
 */

/**
 * \fn Metadata::getLocked(std::string const &tag)
 * \param[in] tag A string used as the key in a map
 *
 * Get the value of the tag key in the map, to allow in-place access
 * to the Metadata contents.
 * This function does not protect against concurrent access, and it is
 * up to the caller to ensure that the lock is held using \a lock()
 */

/**
 * \fn Metadata::setLocked(std::string const &tag, T const &value)
 * \param[in] tag A string used as the key in a map
 * \param[in] value The value to set into the map
 *
 * Set the value to the tag key in the map, to allow in-place access
 * to the Metadata contents.
 * This function does not protect against concurrent access, and it is
 * up to the caller to ensure that the lock is held using \a lock()
 */

/**
 * \fn Metadata::lock()
 * Lock the mutex with the standard classes.
 * e.g. std::lock_guard<RPiController::Metadata> lock(metadata)
 */

/**
 * \fn Metadata::unlock()
 * Unlock the mutex with the standard classes.
 * e.g. std::lock_guard<RPiController::Metadata> lock(metadata)
 */

} /* namespace ipa */

} /* namespace libcamera */

