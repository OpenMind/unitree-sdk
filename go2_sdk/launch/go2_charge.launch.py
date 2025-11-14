from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    go2_charging_node = Node(
        package="my_camera_pkg",
        executable="go2_dock",
        name="dock_charge_node",
        output="screen",
    )
    return LaunchDescription(
        [
            go2_charging_node,
        ]
    )
