import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('go2_sdk')

    channel_type = LaunchConfiguration('channel_type', default=EnvironmentVariable('CHANNEL_TYPE', default_value='serial'))
    serial_port = LaunchConfiguration('serial_port', default=EnvironmentVariable('SERIAL_PORT', default_value='/dev/ttyUSB0'))
    serial_baudrate = LaunchConfiguration('serial_baudrate', default=EnvironmentVariable('SERIAL_BAUDRATE', default_value='115200'))
    frame_id = LaunchConfiguration('frame_id', default=EnvironmentVariable('FRAME_ID', default_value='laser'))
    inverted = LaunchConfiguration('inverted', default=EnvironmentVariable('INVERTED', default_value='false'))
    angle_compensate = LaunchConfiguration('angle_compensate', default=EnvironmentVariable('ANGLE_COMPENSATE', default_value='true'))
    scan_mode = LaunchConfiguration('scan_mode', default=EnvironmentVariable('SCAN_MODE', default_value='Sensitivity'))
    go2_camera_stream_enable = LaunchConfiguration('go2_camera_stream_enable', default=EnvironmentVariable('GO2_CAMERA_STREAM_ENABLE', default_value='true'))
    d435_camera_stream_enable = LaunchConfiguration('d435_camera_stream_enable', default=EnvironmentVariable('D435_CAMERA_STREAM_ENABLE', default_value='true'))

    return LaunchDescription([
        DeclareLaunchArgument(
            'channel_type',
            default_value=channel_type,
            description='Specifying channel type of lidar (can be set via CHANNEL_TYPE environment variable)'),

        DeclareLaunchArgument(
            'serial_port',
            default_value=serial_port,
            description='Specifying usb port to connected lidar (can be set via SERIAL_PORT environment variable)'),

        DeclareLaunchArgument(
            'serial_baudrate',
            default_value=serial_baudrate,
            description='Specifying usb port baudrate to connected lidar (can be set via SERIAL_BAUDRATE environment variable)'),

        DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id,
            description='Specifying frame_id of lidar (can be set via FRAME_ID environment variable)'),

        DeclareLaunchArgument(
            'inverted',
            default_value=inverted,
            description='Specifying whether or not to invert scan data (can be set via INVERTED environment variable)'),

        DeclareLaunchArgument(
            'angle_compensate',
            default_value=angle_compensate,
            description='Specifying whether or not to enable angle_compensate of scan data (can be set via ANGLE_COMPENSATE environment variable)'),

        DeclareLaunchArgument(
            'scan_mode',
            default_value=scan_mode,
            description='Specifying scan mode of lidar (can be set via SCAN_MODE environment variable)'),

        DeclareLaunchArgument(
            'go2_camera_stream_enable',
            default_value=go2_camera_stream_enable,
            description='Enable or disable the go2_camera_stream node (can be set via GO2_CAMERA_STREAM_ENABLE environment variable)'),

        DeclareLaunchArgument(
            'd435_camera_stream_enable',
            default_value=d435_camera_stream_enable,
            description='Enable or disable the d435_camera_stream node (can be set via D435_CAMERA_STREAM_ENABLE environment variable)'),

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
            output='screen',
            respawn=True,
            respawn_delay=2.0
        ),

        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='realsense2_camera_node',
            parameters=[{
                'enable_color': True,
                'enable_depth': True,
                'enable_infra1': False,
                'enable_infra2': False,
                'enable_gyro': False,
                'enable_accel': False,
                'unite_imu_method': 0,
                'rgb_camera.color_profile': '424x240x15',
                'depth_module.depth_profile': '480x270x15'
            }],
            output='screen',
            respawn=True,
            respawn_delay=2.0
        ),

        Node(
            package='go2_sdk',
            executable='d435_obstacle_dector',
            name='d435_obstacle_dector',
            output='screen',
            respawn=True,
            respawn_delay=2.0
        ),

        Node(
            package='go2_sdk',
            executable='om_path',
            name='om_path',
            output='screen',
            respawn=True,
            respawn_delay=2.0
        ),

        Node(
            package='go2_sdk',
            executable='go2_camera_stream',
            name='go2_camera_stream',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            condition=IfCondition(go2_camera_stream_enable)
        ),

        Node(
            package='go2_sdk',
            executable='d435_camera_stream',
            name='d435_camera_stream',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            condition=IfCondition(d435_camera_stream_enable)
        )
    ])
