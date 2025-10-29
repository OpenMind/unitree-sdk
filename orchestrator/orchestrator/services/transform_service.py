import rclpy
import tf2_ros
from tf2_ros import TransformException
from typing import Optional, Dict, Any


class TransformService:
    """
    Handles robot pose transformations and TF operations.
    """

    def __init__(self, node, logger=None):
        """
        Initialize the TransformService.

        Parameters:
        -----------
        node
            ROS2 node instance.
        logger
            Logger instance for logging operations.
        """
        self.node = node
        self.logger = logger
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, node)

    def get_robot_pose_in_map(self) -> Optional[Dict[str, Any]]:
        """
        Get the current robot pose in the map frame.

        Returns:
        -------
        dict or None
            Dictionary with position and orientation, or None if transform fails.
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )

            pose = {
                "position": {
                    "x": transform.transform.translation.x,
                    "y": transform.transform.translation.y,
                    "z": transform.transform.translation.z
                },
                "orientation": {
                    "x": transform.transform.rotation.x,
                    "y": transform.transform.rotation.y,
                    "z": transform.transform.rotation.z,
                    "w": transform.transform.rotation.w
                }
            }

            return pose

        except TransformException as e:
            if self.logger:
                self.logger.error(f"Failed to get transform: {str(e)}")
            return None

    def get_transform(self, target_frame: str, source_frame: str, timeout: float = 1.0) -> Optional[Dict[str, Any]]:
        """
        Get transform between two frames.

        Parameters:
        -----------
        target_frame : str
            Target frame ID.
        source_frame : str
            Source frame ID.
        timeout : float
            Timeout in seconds.

        Returns:
        --------
        dict or None
            Transform data or None if failed.
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=timeout)
            )

            return {
                "translation": {
                    "x": transform.transform.translation.x,
                    "y": transform.transform.translation.y,
                    "z": transform.transform.translation.z
                },
                "rotation": {
                    "x": transform.transform.rotation.x,
                    "y": transform.transform.rotation.y,
                    "z": transform.transform.rotation.z,
                    "w": transform.transform.rotation.w
                },
                "header": {
                    "frame_id": transform.header.frame_id,
                    "stamp": {
                        "sec": transform.header.stamp.sec,
                        "nanosec": transform.header.stamp.nanosec
                    }
                },
                "child_frame_id": transform.child_frame_id
            }

        except TransformException as e:
            if self.logger:
                self.logger.error(f"Failed to get transform from {source_frame} to {target_frame}: {str(e)}")
            return None
