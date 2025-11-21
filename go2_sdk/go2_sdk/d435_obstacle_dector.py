import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point32
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud


class D435ObstacleDector(Node):
    def __init__(self):
        super().__init__("d435_obstacle_dector")
        self.bridge = CvBridge()
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.obstacle_threshold = 0.10  # 10cm above ground
        self.obstacle = []

        self.depth_subscription = self.create_subscription(
            Image,
            "/camera/realsense2_camera_node/depth/image_rect_raw",
            self.depth_callback,
            10,
        )

        self.depth_info = self.create_subscription(
            CameraInfo,
            "/camera/realsense2_camera_node/depth/camera_info",
            self.depth_info_callback,
            10,
        )

        self.obstacle_pub = self.create_publisher(
            PointCloud, "/camera/realsense2_camera_node/depth/obstacle_point", 10
        )

        self.get_logger().info("Intel435ObstacleDector node started")

    def depth_info_callback(self, msg):
        try:
            self.camera_info = msg
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
        except Exception as e:
            self.get_logger().error(f"Error processing depth info: {e}")

    def image_to_world_vectorized(self, depth_image, camera_height=0.45, tilt_angle=55):
        """
        Vectorized conversion from image coordinates to world coordinates
        """
        if self.fx is None or self.fy is None or self.cx is None or self.cy is None:
            self.get_logger().debug("Camera intrinsics not available yet")
            return []

        rows, cols = np.mgrid[
            0 : depth_image.shape[0] : 10, 0 : depth_image.shape[1] : 10
        ]

        depth_values = depth_image[rows, cols]
        valid_mask = depth_values > 0
        if not np.any(valid_mask):
            return []

        rows_valid = rows[valid_mask]
        cols_valid = cols[valid_mask]
        depth_valid = depth_values[valid_mask] / 1000.0  # Convert to meters

        # Camera optical frame: x right, y down, z forward
        cam_x = (cols_valid - self.cx) * depth_valid / self.fx
        cam_y = (rows_valid - self.cy) * depth_valid / self.fy
        cam_z = depth_valid

        # only use world_z to filter obstacles
        points_camera = np.vstack([cam_x, cam_y, cam_z])

        theta = np.radians(tilt_angle)
        R_tilt = np.array(
            [
                [1, 0, 0],
                [0, np.cos(theta), np.sin(theta)],
                [0, -np.sin(theta), np.cos(theta)],
            ]
        )

        R_align = np.array(
            [
                [0, 0, 1],  # Camera Z (forward) -> World X (forward)
                [-1, 0, 0],  # Camera X (right)  -> World Y (left)
                [0, -1, 0],  # Camera Y (down)   -> World Z (up)
            ]
        )

        R_combined = R_align @ R_tilt
        points_world = R_combined @ points_camera

        camera_position_world = np.array([[0], [0], [camera_height]])
        points_world = points_world + camera_position_world

        world_z = points_world[2]  # up

        obstacle_mask = world_z > self.obstacle_threshold
        if not np.any(obstacle_mask):
            return []

        # Only select which points are obstacles, but output coordinates are still in camera frame
        cam_x_filtered = cam_x[obstacle_mask]
        cam_y_filtered = cam_y[obstacle_mask]
        cam_z_filtered = cam_z[obstacle_mask]

        obstacles = []
        for i in range(len(cam_x_filtered)):
            obstacles.append(
                Point32(
                    x=float(cam_x_filtered[i]),
                    y=float(cam_y_filtered[i]),
                    z=float(cam_z_filtered[i]),
                )
            )

        return obstacles

    def depth_callback(self, msg):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

            obstacle = self.image_to_world_vectorized(
                depth_image, camera_height=0.45, tilt_angle=55
            )

            self.obstacle = obstacle
            self.get_logger().debug(f"Detected {len(self.obstacle)} obstacles")

            pc_msg = PointCloud()
            pc_msg.header = msg.header
            pc_msg.points = obstacle
            self.obstacle_pub.publish(pc_msg)

        except Exception as e:
            self.get_logger().error(f"Error processing depth image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = D435ObstacleDector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
