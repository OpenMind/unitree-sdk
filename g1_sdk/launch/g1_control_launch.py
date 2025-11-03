#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('g1_sdk')

    return LaunchDescription([
        # G1 Movement Control Node - converts cmd_vel to G1 locomotion commands
        # Automatically initializes robot on startup
        Node(
            package='g1_sdk',
            executable='cmd_vel_to_g1',
            name='cmd_vel_to_g1',
            output='screen',
            parameters=[{
                'max_linear_x_velocity': 0.5,  # m/s forward/backward
                'max_linear_y_velocity': 0.3,  # m/s lateral
                'max_angular_velocity': 0.5,   # rad/s rotation
            }]
        ),

        # G1 Locomotion Action Node - joystick control for G1 actions
        Node(
            package='g1_sdk',
            executable='g1_loco_action',
            name='g1_loco_action',
            output='screen',
        ),

        # Joystick Node
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),

        # Teleop Twist Joy for G1 - humanoid-specific configuration
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            output='screen',
            parameters=[{
                'axis_linear.x': 1,     # Left stick Y-axis for forward/back
                'axis_linear.y': 0,     # Left stick X-axis for left/right strafe
                'axis_angular.z': 3,    # Right stick X-axis for rotation
                'enable_button': 10,    # RB button (right bumper)
                'scale_linear': 0.6,    # Conservative linear scaling for humanoid
                'scale_angular': 0.6,   # Conservative angular scaling for humanoid
                'scale_linear_turbo': 1.0,
                'scale_angular_turbo': 1.0,
                'enable_turbo_button': 9,  # LB button (left bumper)
                'require_enable_button': True,
            }]
        ),
    ])
