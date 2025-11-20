import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("g1_sdk")
    nav2_config_file = os.path.join(pkg_dir, "config", "nav2_parameters.yaml")
    slam_launch_file = os.path.join(pkg_dir, "launch", "slam_launch.py")

    from launch.actions import TimerAction

    # Include SLAM launch file with localization mode
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_file),
        launch_arguments={"localization": "true"}.items(),
    )

    # Nodes to be launched after delay
    delayed_nodes = [
        Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            name="pointcloud_to_laserscan",
            output="screen",
            remappings=[("cloud_in", "/utlidar/cloud_livox_mid360"), ("scan", "/scan")],
            parameters=[
                {
                    "target_frame": "base_link",
                    "transform_tolerance": 0.01,
                    "min_height": -0.5,
                    "max_height": 3.0,
                    "angle_min": -3.14159,
                    "angle_max": 3.14159,
                    "angle_increment": 0.0087,
                    "scan_time": 0.1,
                    "range_min": 0.5,
                    "range_max": 12.0,
                    "use_inf": True,
                }
            ],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[
                {"use_sim_time": False},
                {"autostart": True},
                {
                    "node_names": [
                        "controller_server",
                        "smoother_server",
                        "planner_server",
                        "behavior_server",
                        "bt_navigator",
                        "waypoint_follower",
                        "velocity_smoother",
                    ]
                },
            ],
        ),
        Node(
            package="nav2_controller",
            executable="controller_server",
            output="screen",
            parameters=[nav2_config_file],
            remappings=[("/cmd_vel", "/cmd_vel")],
        ),
        Node(
            package="nav2_smoother",
            executable="smoother_server",
            name="smoother_server",
            output="screen",
            parameters=[nav2_config_file],
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[nav2_config_file],
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[nav2_config_file],
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[nav2_config_file],
        ),
        Node(
            package="nav2_waypoint_follower",
            executable="waypoint_follower",
            name="waypoint_follower",
            output="screen",
            parameters=[nav2_config_file],
        ),
        Node(
            package="nav2_velocity_smoother",
            executable="velocity_smoother",
            name="velocity_smoother",
            output="screen",
            parameters=[nav2_config_file],
            remappings=[
                ("/cmd_vel", "/cmd_vel_nav"),
                ("/cmd_vel_smoothed", "/cmd_vel"),
            ],
        ),
        Node(
            package="g1_sdk",
            executable="g1_nav2_api",
            name="g1_nav2_api",
            output="screen",
        ),
        Node(
            package="g1_sdk",
            executable="waypoint_manager",
            name="waypoint_manager",
            output="screen",
        ),
    ]

    return LaunchDescription(
        [slam_launch, TimerAction(period=5.0, actions=delayed_nodes)]
    )