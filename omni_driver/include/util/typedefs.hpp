#pragma once

#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include "../include/ros/omni_ros_interface.hpp"

typedef boost::shared_ptr<OmniRosInterface> OmniBasePtr;
typedef boost::unique_lock<boost::shared_mutex>            LockUnique;          ///< The unique lock, used to protect data while writing.
typedef boost::shared_lock<boost::shared_mutex>            LockShared;          ///< The shared lock, used to protect data while reading.
typedef boost::upgrade_lock<boost::shared_mutex>           LockUpgrade;         ///< The upgradeable lock, used to protect data while reading.
typedef boost::upgrade_to_unique_lock<boost::shared_mutex> LockUpgradeToUnique; ///< The upgraded lock, used to protect data while writing.
typedef boost::posix_time::microsec_clock Clock;
typedef boost::posix_time::ptime Time;
typedef boost::posix_time::time_duration TimeDuration;