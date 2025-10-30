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
                "video_device": "/dev/video10",
                "image_width": 640,
                "image_height": 480,
                "framerate": 30.0,
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

