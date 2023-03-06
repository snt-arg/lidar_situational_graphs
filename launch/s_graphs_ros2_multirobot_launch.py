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
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="Namespace for the robot",
            ),
            OpaqueFunction(function=launch_sgraphs),
        ]
    )


def launch_sgraphs(context, *args, **kwargs):
    pkg_dir = get_package_share_directory("s_graphs")
    prefiltering_param_file = os.path.join(pkg_dir, "config", "prefiltering.yaml")
    scan_matching_param_file = os.path.join(pkg_dir, "config", "scan_matching.yaml")
    s_graphs_param_file = os.path.join(pkg_dir, "config", "s_graphs.yaml")
    compute_odom_arg = LaunchConfiguration("compute_odom").perform(context)
    namespace_arg = LaunchConfiguration("namespace").perform(context)
    # log namespace to console
    ns_prefix = str(namespace_arg) + "/" if namespace_arg else ""
    if str(ns_prefix).startswith("/"):
        ns_prefix = ns_prefix[1:]
    print("ns_prefix", ns_prefix)

    prefiltering_cmd = Node(
        package="s_graphs",
        executable="s_graphs_prefiltering_node",
        namespace=namespace_arg,
        parameters=[prefiltering_param_file],
        output="screen",
        remappings=[
            ("velodyne_points", "platform/velodyne_points"),
            ("imu/data", "platform/imu/data"),
        ],
    )

    scan_matching_cmd = Node(
        package="s_graphs",
        executable="s_graphs_scan_matching_odometry_node",
        namespace=namespace_arg,
        parameters=[scan_matching_param_file],
        remappings=[("odom", "platform/odometry")],
        output="screen",
        condition=IfCondition(compute_odom_arg),
    )

    room_segmentation_cmd = Node(
        package="s_graphs",
        executable="s_graphs_room_segmentation_node",
        namespace=namespace_arg,
        parameters=[{"vertex_neigh_thres": 2}],
        output="screen",
    )

    floor_plan_cmd = Node(
        package="s_graphs",
        executable="s_graphs_floor_plan_node",
        namespace=namespace_arg,
        parameters=[{"vertex_neigh_thres": 2}],
        output="screen",
    )

    s_graphs_cmd = Node(
        package="s_graphs",
        executable="s_graphs_node",
        namespace=namespace_arg,
        parameters=[s_graphs_param_file],
        output="screen",
        remappings=[
            ("velodyne_points", "platform/velodyne_points"),
            ("odom", "platform/odometry"),
        ],
    )

    map_keyframe_static_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_keyframe_static_transform",
        arguments=["0.0", 
                   "0.0",
                   "7.0",
                   "0.0", 
                   "0.0", 
                   "0.0",
                   ns_prefix+"map", 
                   ns_prefix+"keyframes_layer"],
        output="screen",
    )

    keyframe_wall_static_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="keyframe_wall_static_transform",
        arguments=[
            "0.0",
            "0.0",
            "8.0",
            "0.0",
            "0.0",
            "0.0",
            ns_prefix+"keyframes_layer",
            ns_prefix+"walls_layer",
        ],
        output="screen",
    )

    wall_room_static_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="wall_room_static_transform",
        arguments=[
            "0.0",
            "0.0",
            "7.0",
            "0.0",
            "0.0",
            "0.0",
            ns_prefix+"walls_layer",
            ns_prefix+"rooms_layer",
        ],
        output="screen",
    )

    room_floor_static_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="room_floor_static_transform",
        arguments=[
            "0.0",
            "0.0",
            "5.0",
            "0.0",
            "0.0",
            "0.0",
            ns_prefix+"rooms_layer",
            ns_prefix+"floors_layer",
        ],
        output="screen",
    )

    return [
        prefiltering_cmd,
        scan_matching_cmd,
        room_segmentation_cmd,
        floor_plan_cmd,
        s_graphs_cmd,
        map_keyframe_static_transform,
        keyframe_wall_static_transform,
        wall_room_static_transform,
        room_floor_static_transform,
    ]
