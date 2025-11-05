import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    g1_jointstate_node = Node(
        package='g1_sdk',
        executable='g1_jointstate',
        name='g1_jointstate',
        output='screen',
    )
    g1_odom_node = Node(
        package='g1_sdk',
        executable='g1_odom',
        name='g1_odom',
        output='screen',
    )
    # Static transform from robot_center to base_link (identity)
    # Relay /dog_odom to /odom for all consumers
    pkg_dir = get_package_share_directory('g1_sdk')

    urdf_file = os.path.join(pkg_dir, 'urdf', 'g1_23dof.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    slam_config_file = os.path.join(pkg_dir, 'config', 'slam.yaml')

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            )
        ])
    )

    return LaunchDescription([
        # Node(
        #     package='joint_state_publisher',
        #     executable='joint_state_publisher',
        #     name='joint_state_publisher',
        #     output='screen',
        # ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            output='screen',
            remappings=[
                ('cloud_in', '/utlidar/cloud_livox_mid360'),  
                ('scan', '/scan')
            ],
            parameters=[{
                'target_frame': 'base_link',  # This should transform the scan
                'transform_tolerance': 0.01,
                'min_height': -0.5,
                'max_height': 3.0,
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'angle_increment': 0.0087,
                'scan_time': 0.1,
                'range_min': 0.5,
                'range_max': 12.0,
                'use_inf': True,
            }]
        ),
        g1_jointstate_node,
        g1_odom_node,
        slam_toolbox_launch,
    ])
