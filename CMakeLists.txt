# SPDX-License-Identifier: BSD-2-Clause
cmake_minimum_required(VERSION 2.8.3)
project(s_graphs)

# Can we use C++17 in indigo?
add_definitions(-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
set(CMAKE_CXX_FLAGS "-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

# pcl 1.7 causes a segfault when it is built with debug mode
set(CMAKE_BUILD_TYPE "RELEASE")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  pcl_ros
  geodesy
  nmea_msgs
  sensor_msgs
  geometry_msgs
  message_generation
  interactive_markers
  ndt_omp
  fast_gicp
)
catkin_python_setup()

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

find_library(VGICP_CUDA_FOUND NAMES fast_vgicp_cuda)
message(STATUS "VGICP_CUDA_FOUND:" ${VGICP_CUDA_FOUND})
if(VGICP_CUDA_FOUND)
  add_definitions(-DUSE_VGICP_CUDA)
endif()

########################
## message generation ##
########################
add_message_files(FILES
  FloorCoeffs.msg
  ScanMatchingStatus.msg
  PointClouds.msg
  RoomData.msg
  RoomsData.msg
  PlaneData.msg
  PlanesData.msg
)

add_service_files(FILES
  SaveMap.srv
  DumpGraph.srv
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs sensor_msgs)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES s_graphs_nodelet
#  CATKIN_DEPENDS pcl_ros roscpp sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(include)
include_directories(
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# nodelets
add_library(s_graphs_prefiltering_nodelet apps/prefiltering_nodelet.cpp)
target_link_libraries(s_graphs_prefiltering_nodelet
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_library(s_graphs_plane_segmentation_nodelet apps/plane_segmentation_nodelet.cpp)
target_link_libraries(s_graphs_plane_segmentation_nodelet
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
add_dependencies(s_graphs_plane_segmentation_nodelet ${PROJECT_NAME}_gencpp {${PROJECT_NAME}_EXPORTED_TARGETS})

add_library(s_graphs_room_segmentation_nodelet apps/room_segmentation_nodelet.cpp)
target_link_libraries(s_graphs_room_segmentation_nodelet
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
add_dependencies(s_graphs_room_segmentation_nodelet ${PROJECT_NAME}_gencpp {${PROJECT_NAME}_EXPORTED_TARGETS})

add_library(s_graphs_floor_detection_nodelet apps/floor_detection_nodelet.cpp)
target_link_libraries(s_graphs_floor_detection_nodelet
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
add_dependencies(s_graphs_floor_detection_nodelet ${PROJECT_NAME}_gencpp {${PROJECT_NAME}_EXPORTED_TARGETS})

add_library(s_graphs_scan_matching_odometry_nodelet
  apps/scan_matching_odometry_nodelet.cpp
  src/s_graphs/registrations.cpp
)
target_link_libraries(s_graphs_scan_matching_odometry_nodelet
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
add_dependencies(s_graphs_scan_matching_odometry_nodelet ${PROJECT_NAME}_gencpp {${PROJECT_NAME}_EXPORTED_TARGETS})

add_library(s_graphs_nodelet
  apps/s_graphs_nodelet.cpp
  src/s_graphs/graph_slam.cpp
  src/s_graphs/keyframe.cpp
  src/s_graphs/map_cloud_generator.cpp
  src/s_graphs/registrations.cpp
  src/s_graphs/information_matrix_calculator.cpp
  src/g2o/robust_kernel_io.cpp
)
target_link_libraries(s_graphs_nodelet
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${G2O_TYPES_DATA}
  ${G2O_CORE_LIBRARY}
  ${G2O_STUFF_LIBRARY}
  ${G2O_SOLVER_PCG}
  ${G2O_SOLVER_CSPARSE}   # be aware of that CSPARSE is released under LGPL
  ${G2O_SOLVER_CHOLMOD}   # be aware of that cholmod is released under GPL
  ${G2O_TYPES_SLAM3D}
  ${G2O_TYPES_SLAM3D_ADDONS}
)
add_dependencies(s_graphs_nodelet ${PROJECT_NAME}_gencpp ${${PROJECT_NAME}_EXPORTED_TARGETS})

catkin_install_python(
  PROGRAMS
    src/${PROJECT_NAME}/bag_player.py
    src/${PROJECT_NAME}/ford2bag.py
    src/${PROJECT_NAME}/map2odom_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)