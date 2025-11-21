from collections import deque
from math import cos, hypot, pi, sin
from typing import Any

import numpy as np
import rclpy
import tf_transformations
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

from om_api.msg import LocalizationPose


class Go2LidarLocalizationNode(Node):
    def __init__(self):
        super().__init__("go2_lidar_localization")

        # Declare parameters
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("laser_frame", "laser")
        self.declare_parameter("laser_topic", "scan")
        self.declare_parameter("velocity_threshold", 0.3)  # m/s
        self.declare_parameter("scan_match_interval", 0.2)  # seconds
        self.declare_parameter(
            "global_localization_particles", 5000
        )  # Number of random samples
        self.declare_parameter(
            "min_confidence_for_prediction", 0.9
        )  # Minimum confidence threshold for pose prediction
        self.declare_parameter(
            "max_consecutive_failures", 5
        )  # Maximum consecutive failures before triggering re-localization
        self.declare_parameter(
            "failure_quality_threshold", 0.9
        )  # Quality threshold below which to consider a failure

        # Get parameters
        self.base_frame = self.get_parameter("base_frame").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.laser_frame = self.get_parameter("laser_frame").value
        self.laser_topic = self.get_parameter("laser_topic").value
        self.velocity_threshold = self.get_parameter("velocity_threshold").value
        self.scan_match_interval = self.get_parameter("scan_match_interval").value
        self.n_particles = self.get_parameter("global_localization_particles").value
        self.min_confidence_for_prediction = self.get_parameter(
            "min_confidence_for_prediction"
        ).value
        self.max_consecutive_failures = self.get_parameter(
            "max_consecutive_failures"
        ).value
        self.failure_quality_threshold = self.get_parameter(
            "failure_quality_threshold"
        ).value

        # Initialize variables
        self.map_msg = None
        self.map_cropped = None
        self.map_temp = None
        self.map_roi_info = {"x_offset": 0, "y_offset": 0, "width": 0, "height": 0}
        self.scan_points = []
        self.free_space_points = []  # Store free space locations for sampling

        self.lidar_x = 250.0
        self.lidar_y = 250.0
        self.lidar_yaw = 0.0
        self.deg_to_rad = pi / 180.0
        self.clear_countdown = -1
        self.scan_count = 0
        self.data_queue = deque(maxlen=10)

        # Global localization state
        self.is_initialized = False
        self.last_match_quality = 0.0
        self.buffered_scan = None  # Store scan for global localization

        # Consecutive failure tracking for automatic re-localization
        self.consecutive_failures = 0
        self.last_failure_time = None

        # Odometry tracking for motion prediction
        self.last_odom_pose = None  # (x, y, yaw)
        self.last_scan_match_time = None
        self.current_velocity = 0.0
        self.last_velocity_check_time = None
        self.last_velocity_pose = None

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            "/map",
            self.map_callback,
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            ),
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            self.laser_topic,
            self.scan_callback,
            10,
        )

        self.initial_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, "initialpose", self.initial_pose_callback, 10
        )

        # Service for triggering global localization
        self.global_loc_service = self.create_service(
            Empty, "/om/global_localization", self.global_localization_callback
        )

        # Publisher for localization pose information
        self.localization_pose_pub = self.create_publisher(
            LocalizationPose, "/om/localization_pose", 10
        )

        # Timer for publishing TF at 30Hz
        self.tf_timer = self.create_timer(1.0 / 30.0, self.pose_tf)

        # Pose estimated flag
        self.is_pose_estimated = False
        self.is_pose_confident = False

        self.get_logger().info("Lidar localization node initialized")
        self.get_logger().info(f"Velocity threshold: {self.velocity_threshold} m/s")
        self.get_logger().info(f"Scan match interval: {self.scan_match_interval} s")
        self.get_logger().info(f"Global localization particles: {self.n_particles}")
        self.get_logger().info(
            f"Minimum confidence for prediction: {self.min_confidence_for_prediction}"
        )
        self.get_logger().info(
            f"Max consecutive failures: {self.max_consecutive_failures}"
        )
        self.get_logger().info(
            f"Failure quality threshold: {self.failure_quality_threshold}"
        )

    def initial_pose_callback(self, msg: PoseWithCovarianceStamped):
        """
        Handle initial pose from RViz or other sources

        Parameters
        ----------
        msg: PoseWithCovarianceStamped
            Incoming initial pose message
        """
        if self.map_msg is None or self.map_msg.info.resolution <= 0:
            self.get_logger().error("Map info invalid or not received")
            return

        # Extract position and orientation from message
        map_x = msg.pose.pose.position.x
        map_y = msg.pose.pose.position.y

        # Convert quaternion to yaw
        orientation = msg.pose.pose.orientation
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = tf_transformations.euler_from_quaternion(quaternion)

        # Convert map coordinates to cropped map grid coordinates
        self.lidar_x = (
            map_x - self.map_msg.info.origin.position.x
        ) / self.map_msg.info.resolution - self.map_roi_info["x_offset"]
        self.lidar_y = (
            map_y - self.map_msg.info.origin.position.y
        ) / self.map_msg.info.resolution - self.map_roi_info["y_offset"]

        self.lidar_yaw = -yaw
        self.clear_countdown = 30
        self.is_initialized = True
        self.is_pose_confident = False

        # Reset consecutive failure counter
        self.consecutive_failures = 0
        self.last_failure_time = None

        # Reset odometry tracking
        self.last_odom_pose = None
        self.last_scan_match_time = None

    def map_callback(self, msg: OccupancyGrid):
        """
        Process incoming map messages

        Parameters
        ----------
        msg: OccupancyGrid
            Incoming map message
        """
        self.map_msg = msg
        self.crop_map()
        self.process_map()
        self.extract_free_space()

    def extract_free_space(self):
        """
        Extract free space points for random sampling
        """
        if self.map_cropped is None:
            return

        # Find free space (value = 0 in occupancy grid)
        # We want to sample from areas that are definitely free
        free_cells = np.where(self.map_cropped == 0)

        if len(free_cells[0]) == 0:
            self.get_logger().warn("No free space found in map!")
            return

        # Store as (x, y) tuples in cropped map coordinates
        self.free_space_points = list(zip(free_cells[1], free_cells[0]))

        self.get_logger().info(
            f"Found {len(self.free_space_points)} free space cells for sampling"
        )

    def global_localization_callback(self, request: Any, response: Any) -> Any:
        """
        Service callback to trigger global localization

        Parameters
        ----------
        request: Any
            Incoming service request
        response: Any
            Service response

        Returns
        -------
        Any
            Service response
        """
        self.get_logger().info("Global localization service called")

        if self.buffered_scan is not None:
            self.perform_global_localization(self.buffered_scan)
        else:
            self.get_logger().warn(
                "No scan available for global localization, will use next scan"
            )
            self.is_initialized = False

        return response

    def evaluate_pose_vectorized(
        self, x: float, y: float, yaw: float, scan_points
    ) -> float:
        """
        Vectorized evaluation of pose - much faster than loop-based version

        Parameters
        ----------
        x: float
            X coordinate in map frame
        y: float
            Y coordinate in map frame
        yaw: float
            Orientation in radians
        scan_points: np.ndarray
            Numpy array of (x, y) points in base frame

        Returns
        -------
        float
            Match score
        """
        # Convert scan_points to numpy array if not already
        if not isinstance(scan_points, np.ndarray):
            scan_points = np.array(scan_points)

        if len(scan_points) == 0:
            return 0

        # Precompute trig values
        cos_yaw = cos(yaw)
        sin_yaw = sin(yaw)

        # Vectorized rotation and translation
        rx = scan_points[:, 0] * cos_yaw - scan_points[:, 1] * sin_yaw + x
        ry = scan_points[:, 0] * sin_yaw + scan_points[:, 1] * cos_yaw

        map_x = rx.astype(int)
        map_y = (y - ry).astype(int)

        # Filter valid coordinates
        valid_mask = (
            (map_x >= 0)
            & (map_x < self.map_temp.shape[1])
            & (map_y >= 0)
            & (map_y < self.map_temp.shape[0])
        )

        # Sum scores only for valid points
        if np.any(valid_mask):
            score = np.sum(self.map_temp[map_y[valid_mask], map_x[valid_mask]])
        else:
            score = 0

        return int(score)

    def perform_global_localization(self, scan_msg: LaserScan):
        """
        Perform global localization by sampling random poses

        Parameters
        ----------
        scan_msg: LaserScan
            Incoming laser scan message for localization
        """
        if len(self.free_space_points) == 0:
            self.get_logger().error("No free space points available for sampling")
            return

        self.get_logger().info(
            f"Starting global localization with {self.n_particles} particles..."
        )

        # Convert scan to points (reuse existing logic)
        scan_points = self.convert_scan_to_points(scan_msg)

        if len(scan_points) == 0:
            self.get_logger().error("No valid scan points for global localization")
            return

        # Convert to numpy array for vectorized operations
        scan_points = np.array(scan_points)

        best_score = 0
        best_pose = None

        # Sample random poses
        sample_indices = np.random.choice(
            len(self.free_space_points),
            min(self.n_particles, len(self.free_space_points)),
            replace=False,
        )

        for idx in sample_indices:
            x, y = self.free_space_points[idx]

            # Try multiple orientations for each position
            for yaw in np.linspace(0, 2 * pi, 8, endpoint=False):
                score = self.evaluate_pose_vectorized(x, y, yaw, scan_points)

                if score > best_score:
                    best_score = score
                    best_pose = (x, y, yaw)

        if best_pose is not None:
            self.lidar_x, self.lidar_y, self.lidar_yaw = best_pose
            self.is_initialized = True
            self.is_pose_confident = False
            self.scan_count = 1

            # Reset consecutive failure tracking
            self.consecutive_failures = 0
            self.last_failure_time = None

            self.last_odom_pose = None
            self.last_scan_match_time = None
            self.last_velocity_pose = None
            self.last_velocity_check_time = None

            self.scan_points = self.convert_scan_to_points(scan_msg)
            if len(self.scan_points) > 0:
                self.match_scan_iterative()

            # Calculate match quality
            min_score_threshold = len(scan_points) * 30

            if best_score < min_score_threshold:
                self.get_logger().warn(
                    f"Low confidence in global localization (score: {best_score} < {min_score_threshold})"
                )
            else:
                self.get_logger().info(
                    f"Global localization successful (score: {best_score} >= {min_score_threshold})"
                )
        else:
            self.get_logger().error("Global localization failed - no valid pose found")

    def convert_scan_to_points(self, msg: LaserScan) -> list:
        """
        Convert laser scan to points in base frame

        Parameters
        ----------
        msg: LaserScan
            Incoming laser scan message

        Returns
        -------
        list
            List of (x, y) points in base frame
        """
        scan_points = []

        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.laser_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
        except (TransformException, Exception) as e:
            self.get_logger().warn(f"TF lookup failed: {str(e)}")
            return scan_points

        # Check if lidar is inverted
        q = transform.transform.rotation
        quaternion = [q.x, q.y, q.z, q.w]
        roll, pitch, yaw = tf_transformations.euler_from_quaternion(quaternion)

        tolerance = 0.1
        lidar_is_inverted = (abs(abs(roll) - pi) < tolerance) and not (
            abs(abs(pitch) - pi) < tolerance
        )

        # Convert scan to points
        angle = msg.angle_min
        tx = transform.transform.translation.x
        ty = transform.transform.translation.y

        for _, r in enumerate(msg.ranges):
            if msg.range_min <= r <= msg.range_max:
                x_laser = r * cos(angle)
                y_laser = -r * sin(angle)

                x_base = x_laser * cos(yaw) - y_laser * sin(yaw) + tx
                y_base = x_laser * sin(yaw) + y_laser * cos(yaw) + ty

                x_grid = x_base / self.map_msg.info.resolution
                y_grid = y_base / self.map_msg.info.resolution

                if lidar_is_inverted:
                    x_grid = -x_grid
                    y_grid = -y_grid

                scan_points.append((x_grid, y_grid))

            angle += msg.angle_increment

        return scan_points

    def evaluate_pose(self, x: float, y: float, yaw: float, scan_points: list) -> float:
        """
        Evaluate how well scan matches at given pose

        Parameters
        -----------
        x: float
            X coordinate in map frame
        y: float
            Y coordinate in map frame
        yaw: float
            Orientation in radians
        scan_points: list
            List of (x, y) points in base frame

        Returns
        -------
        float
            Match score
        """
        score = 0

        for px, py in scan_points:
            # Transform point to map frame
            rx = px * cos(yaw) - py * sin(yaw)
            ry = px * sin(yaw) + py * cos(yaw)

            map_x = int(rx + x)
            map_y = int(y - ry)

            # Check bounds and accumulate gradient value
            if (
                0 <= map_x < self.map_temp.shape[1]
                and 0 <= map_y < self.map_temp.shape[0]
            ):
                score += int(self.map_temp[map_y, map_x])

        return score

    def crop_map(self):
        """
        Crop the map to only include occupied areas plus margin
        """
        if self.map_msg is None:
            return

        info = self.map_msg.info
        width = info.width
        height = info.height

        # Convert map data to numpy array
        data = np.array(self.map_msg.data, dtype=np.int8).reshape((height, width))

        # Copy the actual map data
        map_raw = data.astype(np.uint8)

        # Find occupied cells (value = 100, NOT 1!)
        occupied = np.where(data == 100)

        if len(occupied[0]) == 0:
            self.get_logger().warn("No occupied cells found in map")
            return

        # Find bounding box of occupied area
        y_min, y_max = occupied[0].min(), occupied[0].max()
        x_min, x_max = occupied[1].min(), occupied[1].max()

        # Calculate center
        cen_x = (x_min + x_max) // 2
        cen_y = (y_min + y_max) // 2

        # Add margin
        new_half_width = abs(x_max - x_min) // 2 + 50
        new_half_height = abs(y_max - y_min) // 2 + 50

        new_origin_x = max(0, cen_x - new_half_width)
        new_origin_y = max(0, cen_y - new_half_height)
        new_width = min(new_half_width * 2, width - new_origin_x)
        new_height = min(new_half_height * 2, height - new_origin_y)

        # Crop the map
        self.map_cropped = map_raw[
            new_origin_y : new_origin_y + new_height,
            new_origin_x : new_origin_x + new_width,
        ].copy()

        # Store ROI info
        self.map_roi_info = {
            "x_offset": new_origin_x,
            "y_offset": new_origin_y,
            "width": new_width,
            "height": new_height,
        }

        self.get_logger().info(f"Map cropped to {new_width}x{new_height}")

    def create_gradient_mask(self, size: int) -> np.ndarray:
        """
        Create a gradient mask for obstacle expansion

        Parameters
        ----------
        size : int
            Size of the square mask

        Returns
        -------
        np.ndarray
            Gradient mask
        """
        mask = np.zeros((size, size), dtype=np.uint8)
        center = size // 2

        for y in range(size):
            for x in range(size):
                distance = hypot(x - center, y - center)
                value = int(255 * max(0.0, 1.0 - distance / center))
                mask[y, x] = value

        return mask

    def process_map(self):
        """
        Process map to create gradient around obstacles
        """
        if self.map_cropped is None or self.map_cropped.size == 0:
            return

        self.map_temp = np.zeros_like(self.map_cropped, dtype=np.uint8)
        gradient_mask = self.create_gradient_mask(101)

        # Find all obstacle pixels (value = 100)
        obstacles = np.where(self.map_cropped == 100)

        for y, x in zip(obstacles[0], obstacles[1]):
            # Calculate ROI bounds
            left = max(0, x - 50)
            top = max(0, y - 50)
            right = min(self.map_cropped.shape[1] - 1, x + 50)
            bottom = min(self.map_cropped.shape[0] - 1, y + 50)

            # Apply gradient mask
            mask_left = 50 - (x - left)
            mask_top = 50 - (y - top)
            mask_roi = gradient_mask[
                mask_top : mask_top + (bottom - top + 1),
                mask_left : mask_left + (right - left + 1),
            ]

            region = self.map_temp[top : bottom + 1, left : right + 1]
            self.map_temp[top : bottom + 1, left : right + 1] = np.maximum(
                region, mask_roi
            )

    def check_convergence(self, x: float, y: float, yaw: float) -> bool:
        """
        Check if localization has converged

        Parameters
        ----------
        x : float
            Current x position
        y : float
            Current y position
        yaw : float
            Current yaw orientation

        Returns
        -------
        bool
            True if converged, False otherwise
        """
        if x == 0 and y == 0 and yaw == 0:
            self.data_queue.clear()
            return True

        self.data_queue.append((x, y, yaw))

        if len(self.data_queue) == self.data_queue.maxlen:
            first = self.data_queue[0]
            last = self.data_queue[-1]

            dx = abs(last[0] - first[0])
            dy = abs(last[1] - first[1])
            dyaw = abs(last[2] - first[2])

            # Tighter convergence threshold
            if dx < 2 and dy < 2 and dyaw < 2 * self.deg_to_rad:
                self.data_queue.clear()
                return True

        return False

    def update_velocity(self):
        """
        Calculate current velocity from odometry
        """
        try:
            current_transform = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )

            current_time = self.get_clock().now()

            if (
                self.last_velocity_pose is not None
                and self.last_velocity_check_time is not None
            ):
                dt = (current_time - self.last_velocity_check_time).nanoseconds / 1e9

                if dt > 0:
                    dx = (
                        current_transform.transform.translation.x
                        - self.last_velocity_pose[0]
                    )
                    dy = (
                        current_transform.transform.translation.y
                        - self.last_velocity_pose[1]
                    )
                    self.current_velocity = hypot(dx, dy) / dt

            self.last_velocity_pose = (
                current_transform.transform.translation.x,
                current_transform.transform.translation.y,
            )
            self.last_velocity_check_time = current_time

        except (TransformException, Exception) as e:
            self.get_logger().debug(f"Velocity update failed: {str(e)}")

    def predict_pose_from_odometry(self):
        """
        Use odometry to predict where robot moved since last update
        Only predict when previous estimation is confident
        """
        # Avoid the pose prediction on the global localization first run
        if not self.is_pose_estimated:
            self.is_pose_estimated = True
            return

        # Only predict pose if previous estimation was confident
        if not self.is_pose_confident:
            self.get_logger().info(
                "Skipping pose prediction - previous estimation not confident enough",
                throttle_duration_sec=2.0,
            )
            return

        try:
            # Get current odometry
            current_odom = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )

            if self.last_odom_pose is not None:
                # Calculate odometry delta
                dx_odom = current_odom.transform.translation.x - self.last_odom_pose[0]
                dy_odom = current_odom.transform.translation.y - self.last_odom_pose[1]

                # Get rotation delta
                current_q = current_odom.transform.rotation
                current_quat = [current_q.x, current_q.y, current_q.z, current_q.w]
                _, _, current_yaw = tf_transformations.euler_from_quaternion(
                    current_quat
                )

                dyaw_odom = current_yaw - self.last_odom_pose[2]

                # Normalize angle difference to [-pi, pi]
                while dyaw_odom > pi:
                    dyaw_odom -= 2 * pi
                while dyaw_odom < -pi:
                    dyaw_odom += 2 * pi

                # Convert odom delta to map pixels
                dx_pixels = dx_odom / self.map_msg.info.resolution
                dy_pixels = dy_odom / self.map_msg.info.resolution

                # Apply delta to scan matcher position
                # Rotate delta by current orientation
                self.lidar_x += dx_pixels * cos(-self.lidar_yaw) - dy_pixels * sin(
                    -self.lidar_yaw
                )
                self.lidar_y += dx_pixels * sin(-self.lidar_yaw) + dy_pixels * cos(
                    -self.lidar_yaw
                )
                self.lidar_yaw -= dyaw_odom

                # Normalize yaw to [-pi, pi]
                while self.lidar_yaw > pi:
                    self.lidar_yaw -= 2 * pi
                while self.lidar_yaw < -pi:
                    self.lidar_yaw += 2 * pi

                self.get_logger().debug(
                    f"Predicted pose update: dx={dx_pixels:.2f}, dy={dy_pixels:.2f}, dyaw={dyaw_odom:.3f}",
                    throttle_duration_sec=1.0,
                )

            # Store current pose for next iteration
            current_q = current_odom.transform.rotation
            current_quat = [current_q.x, current_q.y, current_q.z, current_q.w]
            _, _, current_yaw = tf_transformations.euler_from_quaternion(current_quat)

            self.last_odom_pose = (
                current_odom.transform.translation.x,
                current_odom.transform.translation.y,
                current_yaw,
            )

        except (TransformException, Exception) as e:
            self.get_logger().debug(f"Odom prediction failed: {str(e)}")

    def scan_callback(self, msg: LaserScan):
        """
        Process laser scan and perform scan matching

        Parameters
        ----------
        msg: LaserScan
            Incoming laser scan message
        """
        self.buffered_scan = msg

        # Check if map is available
        if self.map_msg is None or self.map_msg.info.resolution <= 0:
            return

        if self.map_cropped is None or self.map_temp is None:
            return

        # If not initialized, trigger global localization
        if not self.is_initialized:
            self.perform_global_localization(msg)
            return

        # Update velocity estimate
        self.update_velocity()

        # Skip scan matching if moving too fast
        if self.current_velocity > self.velocity_threshold:
            self.get_logger().debug(
                f"Skipping scan match - velocity too high: {self.current_velocity:.2f} m/s",
                throttle_duration_sec=1.0,
            )
            return

        # Rate limit scan matching
        current_time = self.get_clock().now()
        if self.last_scan_match_time is not None:
            time_since_last = (
                current_time - self.last_scan_match_time
            ).nanoseconds / 1e9
            if time_since_last < self.scan_match_interval:
                return
        self.last_scan_match_time = current_time

        # Convert scan to points
        self.scan_points = self.convert_scan_to_points(msg)

        if len(self.scan_points) == 0:
            return

        if self.scan_count == 0:
            self.scan_count += 1

        # Predict pose using odometry before scan matching
        self.predict_pose_from_odometry()

        # Perform scan matching
        self.match_scan_iterative()

    def match_scan_iterative(self):
        """
        Perform iterative scan matching until convergence
        """
        max_iterations = 100
        iteration = 0

        while iteration < max_iterations:
            iteration += 1

            offsets = [(0, 0), (2, 0), (-2, 0), (0, 2), (0, -2)]

            if iteration <= 5:
                yaw_offsets = [
                    0,
                    15 * self.deg_to_rad,
                    -15 * self.deg_to_rad,
                    30 * self.deg_to_rad,
                    -30 * self.deg_to_rad,
                    45 * self.deg_to_rad,
                    -45 * self.deg_to_rad,
                    60 * self.deg_to_rad,
                    -60 * self.deg_to_rad,
                    75 * self.deg_to_rad,
                    -75 * self.deg_to_rad,
                    90 * self.deg_to_rad,
                    -90 * self.deg_to_rad,
                    105 * self.deg_to_rad,
                    -105 * self.deg_to_rad,
                    120 * self.deg_to_rad,
                    -120 * self.deg_to_rad,
                    135 * self.deg_to_rad,
                    -135 * self.deg_to_rad,
                    150 * self.deg_to_rad,
                    -150 * self.deg_to_rad,
                    165 * self.deg_to_rad,
                    -165 * self.deg_to_rad,
                    180 * self.deg_to_rad,
                ]
            else:
                # Fine angular adjustment
                yaw_offsets = [
                    0,
                    2 * self.deg_to_rad,
                    -2 * self.deg_to_rad,
                    5 * self.deg_to_rad,
                    -5 * self.deg_to_rad,
                ]

            current_score = self.evaluate_pose(
                self.lidar_x, self.lidar_y, self.lidar_yaw, self.scan_points
            )

            max_sum = current_score
            best_dx, best_dy, best_dyaw = 0, 0, 0

            for yaw_offset in yaw_offsets:
                test_yaw = self.lidar_yaw + yaw_offset

                # Transform all points
                transformed = []
                for px, py in self.scan_points:
                    rx = px * cos(test_yaw) - py * sin(test_yaw)
                    ry = px * sin(test_yaw) + py * cos(test_yaw)
                    transformed.append((rx + self.lidar_x, self.lidar_y - ry))

                # Try all offsets
                for dx, dy in offsets:
                    score = 0
                    for tx, ty in transformed:
                        px = int(tx + dx)
                        py = int(ty + dy)

                        if (
                            0 <= px < self.map_temp.shape[1]
                            and 0 <= py < self.map_temp.shape[0]
                        ):
                            score += int(self.map_temp[py, px])

                    if score > max_sum:
                        max_sum = score
                        best_dx = dx
                        best_dy = dy
                        best_dyaw = yaw_offset

            # If no improvement found, we've converged
            if best_dx == 0 and best_dy == 0 and best_dyaw == 0:
                self.get_logger().debug(
                    f"Converged - no better pose found (score: {current_score})",
                    throttle_duration_sec=1.0,
                )
                break

            # Reject bad matches
            min_score_threshold = len(self.scan_points) * 30
            if max_sum < min_score_threshold:
                self.get_logger().warn(
                    f"Low match score: {max_sum} (threshold: {min_score_threshold}), rejecting update",
                    throttle_duration_sec=1.0,
                )
                break

            # Update position
            self.lidar_x += best_dx
            self.lidar_y += best_dy
            self.lidar_yaw += best_dyaw

            # Check convergence
            if self.check_convergence(self.lidar_x, self.lidar_y, self.lidar_yaw):
                break

        # Store match quality
        match_quality = max_sum / (len(self.scan_points) * 255)
        self.last_match_quality = match_quality

        # Update confidence based on match quality
        self.is_pose_confident = match_quality >= self.min_confidence_for_prediction
        confidence_status = "confident" if self.is_pose_confident else "not confident"

        self.get_logger().debug(
            f"Match quality: {match_quality:.3f}, confidence threshold: {self.min_confidence_for_prediction:.3f} - {confidence_status}",
            throttle_duration_sec=2.0,
        )

        # Check for consecutive failures and trigger automatic re-localization
        self.check_and_handle_consecutive_failures(match_quality)

        # Publish localization pose
        try:
            map_to_base_transform = self.tf_buffer.lookup_transform(
                "map",
                self.base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )

            msg = LocalizationPose()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"

            msg.pose = Pose()
            msg.pose.position.x = map_to_base_transform.transform.translation.x
            msg.pose.position.y = map_to_base_transform.transform.translation.y
            msg.pose.position.z = map_to_base_transform.transform.translation.z

            msg.pose.orientation.x = map_to_base_transform.transform.rotation.x
            msg.pose.orientation.y = map_to_base_transform.transform.rotation.y
            msg.pose.orientation.z = map_to_base_transform.transform.rotation.z
            msg.pose.orientation.w = map_to_base_transform.transform.rotation.w

            msg.match_score = max_sum
            msg.quality_percent = match_quality * 100.0
            msg.num_points = len(self.scan_points)

            self.localization_pose_pub.publish(msg)

            self.get_logger().info(
                f"Published localization pose - score: {max_sum}, quality: {match_quality*100:.2f}%, points: {len(self.scan_points)}, confident: {self.is_pose_confident}"
            )

        except (TransformException, Exception) as e:
            self.get_logger().debug(
                f"Failed to get transform for pose message: {str(e)}"
            )

    def check_and_handle_consecutive_failures(self, match_quality: float):
        """
        Check for consecutive localization failures and trigger automatic re-localization

        Parameters
        ----------
        match_quality: float
            Current match quality score (0.0 to 1.0)
        """
        current_time = self.get_clock().now()

        # Check if this is a failure
        is_failure = match_quality < self.failure_quality_threshold

        if is_failure:
            self.consecutive_failures += 1
            self.last_failure_time = current_time

            self.get_logger().warn(
                f"Localization failure detected (quality: {match_quality:.3f} < {self.failure_quality_threshold:.3f}). "
                f"Consecutive failures: {self.consecutive_failures}/{self.max_consecutive_failures}"
            )

            # Check if we've reached the maximum consecutive failures
            if self.consecutive_failures >= self.max_consecutive_failures:
                self.get_logger().error(
                    f"Maximum consecutive failures ({self.max_consecutive_failures}) reached. "
                    "Triggering automatic re-localization..."
                )

                # Trigger automatic global localization
                if self.buffered_scan is not None:
                    self.get_logger().info("Starting automatic global re-localization")
                    self.perform_global_localization(self.buffered_scan)
                else:
                    self.get_logger().warn(
                        "No scan available for re-localization, will use next scan"
                    )
                    self.is_initialized = False

                # Reset consecutive failure counter
                self.consecutive_failures = 0

        else:
            # Success - reset consecutive failure counter
            if self.consecutive_failures > 0:
                self.get_logger().info(
                    f"Localization recovered after {self.consecutive_failures} consecutive failures"
                )
                self.consecutive_failures = 0
                self.last_failure_time = None

    def pose_tf(self):
        """
        Publish map to odom transform
        """
        if self.scan_count == 0 or self.map_cropped is None:
            return

        if self.map_msg is None or self.map_msg.info.resolution <= 0:
            return

        if not self.is_initialized:
            return

        # Convert from cropped map pixels to full map meters
        full_map_pixel_x = self.lidar_x + self.map_roi_info["x_offset"]
        full_map_pixel_y = self.lidar_y + self.map_roi_info["y_offset"]

        x_in_map = (
            full_map_pixel_x * self.map_msg.info.resolution
            + self.map_msg.info.origin.position.x
        )
        y_in_map = (
            full_map_pixel_y * self.map_msg.info.resolution
            + self.map_msg.info.origin.position.y
        )
        yaw_in_map = -self.lidar_yaw

        # Get odom to base transform
        try:
            odom_to_base_msg = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
        except (TransformException, Exception):
            return

        # Map to base transform
        map_to_base = tf_transformations.quaternion_matrix(
            tf_transformations.quaternion_from_euler(0, 0, yaw_in_map)
        )
        map_to_base[0, 3] = x_in_map
        map_to_base[1, 3] = y_in_map

        # Odom to base transform
        odom_q = odom_to_base_msg.transform.rotation
        odom_quat = [odom_q.x, odom_q.y, odom_q.z, odom_q.w]
        odom_to_base = tf_transformations.quaternion_matrix(odom_quat)
        odom_to_base[0, 3] = odom_to_base_msg.transform.translation.x
        odom_to_base[1, 3] = odom_to_base_msg.transform.translation.y

        # Calculate map to odom: T_map_odom = T_map_base * T_base_odom
        # T_base_odom = inverse(T_odom_base)
        base_to_odom = tf_transformations.inverse_matrix(odom_to_base)
        map_to_odom = np.dot(map_to_base, base_to_odom)

        # Extract translation and rotation
        map_to_odom_trans = tf_transformations.translation_from_matrix(map_to_odom)
        map_to_odom_quat = tf_transformations.quaternion_from_matrix(map_to_odom)

        # Publish transform
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = self.odom_frame

        t.transform.translation.x = map_to_odom_trans[0]
        t.transform.translation.y = map_to_odom_trans[1]
        t.transform.translation.z = 0.0

        t.transform.rotation.x = map_to_odom_quat[0]
        t.transform.rotation.y = map_to_odom_quat[1]
        t.transform.rotation.z = map_to_odom_quat[2]
        t.transform.rotation.w = map_to_odom_quat[3]

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = Go2LidarLocalizationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
