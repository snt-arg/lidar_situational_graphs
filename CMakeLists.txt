# SPDX-License-Identifier: BSD-2-Clause
cmake_minimum_required(VERSION 3.5)
project(s_graphs)

# Can we use C++17 in indigo?
add_definitions(-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
set(CMAKE_CXX_FLAGS "-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# pcl 1.7 causes a segfault when it is built with debug mode
if (NOT CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
  set(CMAKE_BUILD_TYPE "RELEASE")
endif()

find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(geodesy REQUIRED)
find_package(nmea_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(reasoning_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(ndt_omp REQUIRED)
find_package(fast_gicp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(message_filters REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(backward_ros REQUIRED)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

message(STATUS "PCL_INCLUDE_DIRS:" ${PCL_INCLUDE_DIRS})
message(STATUS "PCL_LIBRARY_DIRS:" ${PCL_LIBRARY_DIRS})
message(STATUS "PCL_DEFINITIONS:" ${PCL_DEFINITIONS})

find_package(G2O REQUIRED)
include_directories(SYSTEM ${G2O_INCLUDE_DIR} ${G2O_INCLUDE_DIRS})
link_directories(${G2O_LIBRARY_DIRS})
# link_libraries(${G2O_LIBRARIES})

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

find_package(Torch REQUIRED)
include_directories(${TORCH_INCLUDE_DIRS})

find_library(VGICP_CUDA_FOUND NAMES fast_vgicp_cuda)
message(STATUS "VGICP_CUDA_FOUND:" ${VGICP_CUDA_FOUND})
if(VGICP_CUDA_FOUND)
  add_definitions(-DUSE_VGICP_CUDA)
endif()

########################
## message generation ##
########################

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PointClouds.msg"
  "msg/PlaneData.msg"
  "msg/PlanesData.msg"
  "msg/RoomData.msg"
  "msg/RoomsData.msg"
  "msg/FloorCoeffs.msg"
  "msg/ScanMatchingStatus.msg"
  "msg/WallData.msg"
  "msg/WallsData.msg"
  "srv/DumpGraph.srv"
  "srv/SaveMap.srv"
  "srv/LoadGraph.srv"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs sensor_msgs visualization_msgs
)

###########
## Build ##
###########
include_directories(include)
include_directories(
  ${PCL_INCLUDE_DIRS}
)

set(DEPENDENCIES 
    rclcpp
    sensor_msgs 
    geometry_msgs 
    nav_msgs 
    ndt_omp 
    fast_gicp 
    tf2 
    tf2_ros
    pcl_ros 
    visualization_msgs 
    message_filters 
    geographic_msgs
    nmea_msgs
    geodesy
    reasoning_msgs)

SET(G2O_COMMON_LIBS 
  ${PCL_LIBRARIES}
  ${G2O_TYPES_DATA}
  ${G2O_CORE_LIBRARY}
  ${G2O_STUFF_LIBRARY}
  ${G2O_SOLVER_PCG}
  ${G2O_SOLVER_CSPARSE}   # be aware of that CSPARSE is released under LGPL
  ${G2O_SOLVER_CHOLMOD}   # be aware of that cholmod is released under GPL
  ${G2O_TYPES_SLAM3D}
  ${G2O_TYPES_SLAM3D_ADDONS})

FILE(GLOB_RECURSE G2O_S_GRAPHS_SOURCE_FILES 
  ${CMAKE_CURRENT_SOURCE_DIR}/src/g2o/*.cpp
)

FILE(GLOB_RECURSE S_GRAPHS_SOURCE_FILES 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/s_graphs/*.cpp
)

add_library(g2o_sgraphs 
  ${G2O_S_GRAPHS_SOURCE_FILES}
  )
target_link_libraries(g2o_sgraphs
                    ${G2O_COMMON_LIBS}
                    ${PCL_LIBRARIES}
                    )

add_library(s_graphs_core_lib   
    ${S_GRAPHS_SOURCE_FILES}
)
target_link_libraries(s_graphs_core_lib 
                      g2o_sgraphs
                      ${G2O_COMMON_LIBS}
                      ${PCL_LIBRARIES}
                      )
target_include_directories(s_graphs_core_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_target_dependencies(s_graphs_core_lib ${DEPENDENCIES})                   
rosidl_target_interfaces(s_graphs_core_lib ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_export_targets(export_s_graph_lib HAS_LIBRARY_TARGET)
ament_export_dependencies(${DEPENDENCIES})
ament_export_include_directories(include)
ament_export_libraries(s_graphs_core_lib)


add_executable(s_graphs_node
  apps/s_graphs_node.cpp
)
ament_target_dependencies(s_graphs_node ${DEPENDENCIES})
target_link_libraries(s_graphs_node
  s_graphs_core_lib
)
rosidl_target_interfaces(s_graphs_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# nodes
add_executable(s_graphs_prefiltering_node apps/prefiltering_node.cpp)
ament_target_dependencies(s_graphs_prefiltering_node ${DEPENDENCIES})
target_link_libraries(s_graphs_prefiltering_node
  ${PCL_LIBRARIES}
)

add_executable(s_graphs_room_segmentation_node 
  apps/room_segmentation_node.cpp 
)
ament_target_dependencies(s_graphs_room_segmentation_node ${DEPENDENCIES})
target_link_libraries(s_graphs_room_segmentation_node
  s_graphs_core_lib
  ${PCL_LIBRARIES}
)
rosidl_target_interfaces(s_graphs_room_segmentation_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(s_graphs_floor_plan_node
  apps/floor_plan_node.cpp
)
ament_target_dependencies(s_graphs_floor_plan_node ${DEPENDENCIES})
target_link_libraries(s_graphs_floor_plan_node
  s_graphs_core_lib  
  ${PCL_LIBRARIES}
)
rosidl_target_interfaces(s_graphs_floor_plan_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(s_graphs_scan_matching_odometry_node apps/scan_matching_odometry_node.cpp)
ament_target_dependencies(s_graphs_scan_matching_odometry_node ${DEPENDENCIES}) 
target_link_libraries(s_graphs_scan_matching_odometry_node
  ${PCL_LIBRARIES}
  s_graphs_core_lib
)
rosidl_target_interfaces(s_graphs_scan_matching_odometry_node ${PROJECT_NAME} "rosidl_typesupport_cpp")


#############
## Install ##
#############

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS s_graphs_core_lib g2o_sgraphs
  EXPORT export_s_graph_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS  
  s_graphs_prefiltering_node s_graphs_scan_matching_odometry_node s_graphs_room_segmentation_node s_graphs_floor_plan_node s_graphs_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})
install(
  PROGRAMS
    ${PROJECT_NAME}/map2odom_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  # the following lines skip linters
  set(ament_cmake_cppcheck_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(testPlane test/testPlane.cpp)
  target_link_libraries(testPlane s_graphs_core_lib ${PCL_LIBRARIES} ${G2O_LIBRARIES})

  ament_add_gtest(testRoom test/testRoom.cpp)
  target_link_libraries(testRoom s_graphs_core_lib ${PCL_LIBRARIES} ${G2O_LIBRARIES})

  # ament_add_gtest(testRoomCentreCompute test/testRoomCentreCompute.cpp src/s_graphs/room_utils.cpp)
  ament_add_gtest(testRoomCentreCompute test/testRoomCentreCompute.cpp)
  target_link_libraries(testRoomCentreCompute s_graphs_core_lib ${PCL_LIBRARIES} ${G2O_LIBRARIES})

  install(TARGETS
    testPlane testRoom testRoomCentreCompute DESTINATION test/${PROJECT_NAME})
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include) 
ament_package()