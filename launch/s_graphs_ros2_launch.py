import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python import get_package_share_directory
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "compute_odom",
                default_value="true",
                description="Flag to compute the odometry",
            ),
            DeclareLaunchArgument(
                "env",
                default_value="real",
                description="Flag to get the environment type real or sim",
            ),
            OpaqueFunction(function=launch_sgraphs),
        ]
    )


def launch_sgraphs(context, *args, **kwargs):
    pkg_dir = get_package_share_directory("s_graphs")
    prefiltering_param_file = os.path.join(pkg_dir, "config", "prefiltering.yaml")
    scan_matching_param_file = os.path.join(pkg_dir, "config", "scan_matching.yaml")
    s_graphs_param_file = os.path.join(pkg_dir, "config", "s_graphs.yaml")

    prefiltering_cmd = Node(
        package="s_graphs",
        executable="s_graphs_prefiltering_node",
        parameters=[prefiltering_param_file],
        output="screen",
        remappings=[
            ("velodyne_points", "/platform/velodyne_points"),
            ("imu/data", "/platform/imu/data"),
        ],
    )

    compute_odom_arg = LaunchConfiguration("compute_odom").perform(context)

    scan_matching_cmd = Node(
        package="s_graphs",
        executable="s_graphs_scan_matching_odometry_node",
        parameters=[scan_matching_param_file],
        remappings=[("/odom", "/platform/odometry")],
        output="screen",
        condition=IfCondition(compute_odom_arg),
    )

    room_segmentation_cmd = Node(
        package="s_graphs",
        executable="s_graphs_room_segmentation_node",
        parameters=[{"vertex_neigh_thres": 2}],
        output="screen",
    )

    floor_plan_cmd = Node(
        package="s_graphs",
        executable="s_graphs_floor_plan_node",
        parameters=[{"vertex_neigh_thres": 2}],
        output="screen",
    )

    s_graphs_cmd = Node(
        package="s_graphs",
        executable="s_graphs_node",
        parameters=[s_graphs_param_file],
        output="screen",
        remappings=[
            ("velodyne_points", "/platform/velodyne_points"),
            ("/odom", "/platform/odometry"),
        ],
    )

    return [
        prefiltering_cmd,
        scan_matching_cmd,
        room_segmentation_cmd,
        floor_plan_cmd,
        s_graphs_cmd,
    ]
