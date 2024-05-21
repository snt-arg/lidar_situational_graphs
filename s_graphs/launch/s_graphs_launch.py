import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python import get_package_share_directory
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition, LaunchConfigurationEquals

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
            DeclareLaunchArgument(
                "room_segmentation",
                default_value="new",
                description="Algorithm used for room segmentation",
            ),
            DeclareLaunchArgument(
                "debug_mode",
                default_value="false",
                description="Run s_graphs node in debugging mode",
            ),
            OpaqueFunction(function=launch_sgraphs),
        ]
    )

def launch_reasoning():
    reasoning_dir = get_package_share_directory('situational_graphs_reasoning')
    reasoning_launch_file = os.path.join(reasoning_dir, "launch", "situational_graphs_reasoning.launch.py")
    reasoning_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(reasoning_launch_file))

    return reasoning_launch


def launch_sgraphs(context, *args, **kwargs):
    pkg_dir = get_package_share_directory("lidar_s_graphs")
    prefiltering_param_file = os.path.join(pkg_dir, "config", "prefiltering.yaml")
    scan_matching_param_file = os.path.join(pkg_dir, "config", "scan_matching.yaml")
    s_graphs_param_file = os.path.join(pkg_dir, "config", "s_graphs.yaml")

    env_arg = LaunchConfiguration("env").perform(context)
    compute_odom_arg = LaunchConfiguration("compute_odom").perform(context)
    namespace_arg = LaunchConfiguration("namespace").perform(context)
    room_segmentation_arg =  LaunchConfiguration("room_segmentation").perform(context)
    debug_mode_arg =  LaunchConfiguration("debug_mode").perform(context)


    ns_prefix = str(namespace_arg) + "/" if namespace_arg else ""
    if str(ns_prefix).startswith("/"):
        ns_prefix = ns_prefix[1:]

    base_link_frame = "body"
    if env_arg == "sim":
        base_link_frame = "base_footprint"

    prefiltering_cmd = Node(
        package="lidar_s_graphs",
        executable="s_graphs_prefiltering_node",
        namespace=namespace_arg,
        parameters=[{prefiltering_param_file}, {"base_link_frame": base_link_frame}],
        output="screen",
        remappings=[
            ("velodyne_points", "platform/velodyne_points"),
            ("imu/data", "platform/imu/data"),
        ],
    )

    scan_matching_cmd = Node(
        package="lidar_s_graphs",
        executable="s_graphs_scan_matching_odometry_node",
        namespace=namespace_arg,
        parameters=[scan_matching_param_file],
        remappings=[("odom", "platform/odometry")],
        output="screen",
        condition=IfCondition(compute_odom_arg),
    )

    if room_segmentation_arg == "old":
        room_segmentation_cmd = Node(
            package="lidar_s_graphs",
            executable="s_graphs_room_segmentation_node",
            namespace=namespace_arg,
            parameters=[{"vertex_neigh_thres": 2}],
            output="screen",
        )
    else:
      reasoning_launch = launch_reasoning()

    floor_plan_cmd = Node(
        package="lidar_s_graphs",
        executable="s_graphs_floor_plan_node",
        namespace=namespace_arg,
        parameters=[{"vertex_neigh_thres": 2}],
        output="screen",
    )

    s_graphs_cmd = Node(
        package="lidar_s_graphs",
        executable="s_graphs_node",
        namespace=namespace_arg,
        parameters=[s_graphs_param_file],
        output="screen",
        prefix=["gdbserver localhost:3000"] if debug_mode_arg == "true" else None,
        remappings=[
            ("velodyne_points", "platform/velodyne_points"),
            ("odom", "platform/odometry"),
        ],
    )

    map_keyframe_static_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_keyframe_static_transform",
        arguments=[
            "0.0",
            "0.0",
            "7.0",
            "0.0",
            "0.0",
            "0.0",
            ns_prefix + "map",
            ns_prefix + "keyframes_layer",
        ],
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
            ns_prefix + "keyframes_layer",
            ns_prefix + "walls_layer",
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
            ns_prefix + "walls_layer",
            ns_prefix + "rooms_layer",
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
            ns_prefix + "rooms_layer",
            ns_prefix + "floors_layer",
        ],
        output="screen",
    )

    return [
        prefiltering_cmd,
        scan_matching_cmd,
        room_segmentation_cmd if room_segmentation_arg == "old" else reasoning_launch,
        floor_plan_cmd,
        s_graphs_cmd,
        map_keyframe_static_transform,
        keyframe_wall_static_transform,
        wall_room_static_transform,
        room_floor_static_transform,
    ]
