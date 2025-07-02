import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('go2_sdk')

    slam_config_file = os.path.join(pkg_dir, 'config', 'slam.yaml')
    nav2_config_file = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')

    channel_type = LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200')
    frame_id = LaunchConfiguration('frame_id', default='laser')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Sensitivity')
    use_nav2 = LaunchConfiguration('use_nav2', default='true')
    map_yaml_file = LaunchConfiguration('map_yaml_file', default='/home/jan/Documents/GitHub/unitree_go2_rplidar_slam/office.data')

    return LaunchDescription([
        DeclareLaunchArgument(
            'channel_type',
            default_value=channel_type,
            description='Specifying channel type of lidar'),

        DeclareLaunchArgument(
            'serial_port',
            default_value=serial_port,
            description='Specifying usb port to connected lidar'),

        DeclareLaunchArgument(
            'serial_baudrate',
            default_value=serial_baudrate,
            description='Specifying usb port baudrate to connected lidar'),

        DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id,
            description='Specifying frame_id of lidar'),

        DeclareLaunchArgument(
            'inverted',
            default_value=inverted,
            description='Specifying whether or not to invert scan data'),

        DeclareLaunchArgument(
            'angle_compensate',
            default_value=angle_compensate,
            description='Specifying whether or not to enable angle_compensate of scan data'),

        DeclareLaunchArgument(
            'scan_mode',
            default_value=scan_mode,
            description='Specifying scan mode of lidar'),

        DeclareLaunchArgument(
            'use_nav2',
            default_value=use_nav2,
            description='Whether to launch Nav2 navigation stack'),

        DeclareLaunchArgument(
            'map_yaml_file',
            default_value=map_yaml_file,
            description='Full path to map yaml file (leave empty for SLAM mode)'),

        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{
                'channel_type': channel_type,
                'serial_port': serial_port,
                'serial_baudrate': serial_baudrate,
                'frame_id': frame_id,
                'inverted': inverted,
                'angle_compensate': angle_compensate
            }],
            output='screen'),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_laser',
            arguments=['--x', '0', '--y', '0', '--z', '0',
                    '--roll', '0', '--pitch', '0', '--yaw', '3.14159',
                    '--frame-id', 'base_link', '--child-frame-id', 'laser'],
            output='screen'
        ),

        Node(
            package='go2_sdk',
            executable='pose_to_tf',
        ),

        Node(
            package='go2_sdk',
            executable='cmd_vel_to_go2',
            name='cmd_vel_to_go2',
            output='screen',
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
                'scale_angular.z': 1.0
            }]
        ),

        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[slam_config_file],
            output='screen'
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

        # Node(
        #     package='nav2_lifecycle_manager',
        #     executable='lifecycle_manager',
        #     name='lifecycle_manager_localization',
        #     output='screen',
        #     parameters=[{'use_sim_time': False},
        #                {'autostart': True},
        #                {'node_names': ['amcl']}]
        # ),

        # Node(
        #     package='nav2_amcl',
        #     executable='amcl',
        #     name='amcl',
        #     output='screen',
        #     parameters=[nav2_config_file]
        # )
    ])
