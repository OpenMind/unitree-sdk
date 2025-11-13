
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('g1_sdk')

    # Load URDF file
    urdf_file = os.path.join(pkg_dir, 'urdf', 'g1_23dof.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        Node(
            package='g1_sdk',
            executable='cmd_vel_to_g1',
            name='cmd_vel_to_g1',
            output='screen',
        ),

        Node(
            package='g1_sdk',
            executable='g1_loco_action',
            name='g1_loco_action',
            output='screen',
        ),

        Node(
            package='g1_sdk',
            executable='g1_jointstate',
            name='g1_jointstate',
            output='screen',
        ),

        Node(
            package='g1_sdk',
            executable='g1_odom',
            name='g1_odom',
            output='screen',
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),

        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),

        # RB is the enable button
        # The left joystick controls linear movement
        # The right joystick controls angular movement
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            output='screen',
            parameters=[{
                'axis_linear.x': 1,
                'axis_linear.y': 0,
                'axis_angular.z': 3,
                'enable_button': 7,
                'scale_linear.x': 0.5,
                'scale_angular.z': 1.0,
                'enable_turbo_button': 6,
                'scale_turbo_linear.x': 1.5,
                'scale_turbo_angular.z': 2.0,
            }]
        ),
    ])
