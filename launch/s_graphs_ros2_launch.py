# Copyright 2021 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    pkg_dir = get_package_share_directory('s_graphs')
    prefilitering_param_file = os.path.join(
        pkg_dir, 'config', 'prefiltering.yaml')

    prefiltering_cmd = Node(package='s_graphs', executable='s_graphs_prefiltering_node', parameters=[prefilitering_param_file], output='screen', remappings=[('velodyne_points', '/platform/velodyne_points'),
                                                                                                                                                             ('imu/data', '/platform/imu/data')]
                            )

    pkg_dir = get_package_share_directory('s_graphs')
    scan_matching_param_file = os.path.join(
        pkg_dir, 'config', 'scan_matching.yaml')

    scan_matching_cmd = Node(package='s_graphs', executable='s_graphs_scan_matching_odometry_node', parameters=[scan_matching_param_file], output='screen'
                             )

    pkg_dir = get_package_share_directory('s_graphs')
    room_segmentation_cmd = Node(package='s_graphs', executable='s_graphs_room_segmentation_node', parameters=[{"vertex_neigh_thres": 2}], output='screen'
                                 )

    pkg_dir = get_package_share_directory('s_graphs')
    floor_plan_cmd = Node(package='s_graphs', executable='s_graphs_floor_plan_node', parameters=[{"vertex_neigh_thres": 2}], output='screen'
                          )

    ld = LaunchDescription()
    ld.add_action(prefiltering_cmd)
    ld.add_action(scan_matching_cmd)
    ld.add_action(room_segmentation_cmd)
    ld.add_action(floor_plan_cmd)

    return ld
