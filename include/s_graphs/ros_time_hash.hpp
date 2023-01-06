// SPDX-License-Identifier: BSD-2-Clause

#ifndef ROS_TIME_HASH_HPP
#define ROS_TIME_HASH_HPP

#include <boost/functional/hash.hpp>
#include <rclcpp/rclcpp.hpp>
#include <unordered_map>

/**
 * @brief Hash calculation for rclcpp::Time
 */
class RosTimeHash {
 public:
  size_t operator()(const rclcpp::Time& val) const {
    size_t seed = 0;
    boost::hash_combine(seed, val.seconds());
    boost::hash_combine(seed, val.nanoseconds());
    return seed;
  }
};

#endif  // ROS_TIME_HASHER_HPP
