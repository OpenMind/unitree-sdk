from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to your camera_info.yaml
    config_path = os.path.join(
        get_package_share_directory('unitree_go2_auto_dock'),
        'launch',
        'usb_camera_calibrated.yaml'
    )

    return LaunchDescription([

        # USB Camera node
        Node(
            package="usb_cam",
            executable="usb_cam_node_exe",
            name="camera",
            namespace="camera",
            parameters=[{
                "video_device": "/dev/video2",  
                "image_width": 640,
                "image_height": 480,
                "framerate": 30.0,
                "camera_name": "narrow_stereo",
                "camera_info_url": f"file:///home/openmind/camera_ws/src/unitree_go2_auto_dock/config/1080_logi_cam/camera_info.yaml"
            }]
        ),

        # Republish to compressed
        Node(
            package="image_transport",
            executable="republish",
            name="image_republisher",
            arguments=[
                "raw", "compressed",
                "--ros-args", "-r", "in:=/camera/image_raw",
                "-r", "out:=/camera/image_raw/compressed"
            ]
        )
    ])
