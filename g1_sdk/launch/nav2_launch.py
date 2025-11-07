import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('g1_sdk')

    urdf_file = os.path.join(pkg_dir, 'urdf', 'g1_23dof.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    nav2_config_file = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    map_yaml_file = LaunchConfiguration('map_yaml_file', default=EnvironmentVariable('MAP_YAML_FILE', default_value=''))

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_yaml_file',
            description='Full path to map yaml file (leave empty for SLAM mode)'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
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
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            output='screen',
            remappings=[
                ('cloud_in', '/utlidar/cloud_livox_mid360'),
                ('scan', '/scan')
            ],
            parameters=[{
                'target_frame': 'base_link',
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

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': False},
                       {'autostart': True},
                       {'node_names': ['controller_server',
                                      'smoother_server',
                                      'planner_server',
                                      'behavior_server',
                                      'bt_navigator',
                                      'waypoint_follower',
                                      'velocity_smoother']}]
        ),

        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[nav2_config_file],
            remappings=[('/cmd_vel', '/cmd_vel')]
        ),

        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            parameters=[nav2_config_file]
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_config_file]
        ),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav2_config_file]
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_config_file]
        ),

        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[nav2_config_file]
        ),

        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[nav2_config_file],
            remappings=[('/cmd_vel', '/cmd_vel_nav'),
                       ('/cmd_vel_smoothed', '/cmd_vel')]
        ),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'yaml_filename': map_yaml_file
            }]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[
                {'use_sim_time': False},
                {'autostart': True},
                {'node_names': ['map_server', 'amcl']}]
        ),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_config_file]
        )
    ])