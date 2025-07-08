#pragma once

// ROS2 Compatibility Header for tf2 libraries
// Handles .h vs .hpp extension differences between ROS2 versions

#ifdef __has_include
    // tf2_geometry_msgs
    #if __has_include("tf2_geometry_msgs/tf2_geometry_msgs.hpp")
        #include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
    #else
        #include "tf2_geometry_msgs/tf2_geometry_msgs.h"
    #endif
    
    // tf2_eigen
    #if __has_include("tf2_eigen/tf2_eigen.hpp")
        #include "tf2_eigen/tf2_eigen.hpp"
    #else
        #include "tf2_eigen/tf2_eigen.h"
    #endif
    
    // tf2_sensor_msgs
    #if __has_include("tf2_sensor_msgs/tf2_sensor_msgs.hpp")
        #include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
    #else
        #include "tf2_sensor_msgs/tf2_sensor_msgs.h"
    #endif
    
#else
    // Fallback for older compilers that don't support __has_include
    #include "tf2_geometry_msgs/tf2_geometry_msgs.h"
    #include "tf2_eigen/tf2_eigen.h"
    #include "tf2_sensor_msgs/tf2_sensor_msgs.h"
#endif
