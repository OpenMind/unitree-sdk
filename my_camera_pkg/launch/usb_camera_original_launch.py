from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # USB Camera node
        Node(
            package="usb_cam",
            executable="usb_cam_node_exe",
            name="camera",
            namespace="camera",
            parameters=[{
                "video_device": "/dev/video0",
                "image_width": 640,
                "image_height": 480,
                "framerate": 30.0,
            }]
        ),
    ])

