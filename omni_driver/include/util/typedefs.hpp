#pragma once

#include "../include/driver/omni_ethernet.hpp"

typedef boost::shared_ptr<boost::thread> ThreadPtr;
typedef boost::unique_lock<boost::shared_mutex>            LockUnique;          ///< The unique lock, used to protect data while writing.
typedef boost::shared_lock<boost::shared_mutex>            LockShared;          ///< The shared lock, used to protect data while reading.
typedef boost::upgrade_lock<boost::shared_mutex>           LockUpgrade;         ///< The upgradeable lock, used to protect data while reading.
typedef boost::upgrade_to_unique_lock<boost::shared_mutex> LockUpgradeToUnique; ///< The upgraded lock, used to protect data while writing.