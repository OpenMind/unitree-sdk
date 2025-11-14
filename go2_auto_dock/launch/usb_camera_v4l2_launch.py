from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # Camera node (v4l2)
            Node(
                package="v4l2_camera",
                executable="v4l2_camera_node",
                name="camera",
                namespace="camera",
                parameters=[
                    {
                        "video_device": "/dev/video0",  # adjust if your cam is on another device
                        "image_size": [640, 480],  # resolution
                        "time_per_frame": [1, 30],  # 30 FPS
                        "camera_frame_id": "camera_frame",
                        "use_dma_buffer": False,  # important for Jetson
                    }
                ],
            ),
            # Republish raw -> compressed
            Node(
                package="image_transport",
                executable="republish",
                name="image_republisher",
                arguments=[
                    "raw",
                    "--ros-args",
                    "-r",
                    "in:=/camera/image_raw",
                    "compressed",
                    "--ros-args",
                    "-r",
                    "out:=/camera/image_raw/compressed",
                ],
            ),
        ]
    )
