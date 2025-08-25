from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='orchestrator',
            executable='orchestrator_api',
            name='orchestrator_api',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
        ),

        Node(
            package='orchestrator',
            executable='orchestrator_cloud',
            name='orchestrator_cloud',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
        )
    ])
