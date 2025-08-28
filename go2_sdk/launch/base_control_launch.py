from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('go2_sdk')

    return LaunchDescription([
        Node(
            package='go2_sdk',
            executable='cmd_vel_to_go2',
            name='cmd_vel_to_go2',
            output='screen',
        ),

        Node(
            package='go2_sdk',
            executable='go2_sport_action',
            name='go2_sport_action',
            output='screen',
        ),

        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),
    ])
