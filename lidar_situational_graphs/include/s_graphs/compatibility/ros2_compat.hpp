#ifndef ROS2_COMPATIBILITY_HPP
#define ROS2_COMPATIBILITY_HPP

#ifdef USE_HPP_HEADERS
// ROS2 Jazzy and Rolling use .hpp extensions
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#else
// Older ROS2 and ROS1 distributions use .h extensions
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#endif

#endif  // ROS2_COMPATIBILITY_HPP