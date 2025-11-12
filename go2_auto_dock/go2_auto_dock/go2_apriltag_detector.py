import os
import time

import apriltag
import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32MultiArray
from tf2_ros import TransformBroadcaster


class AprilTagNode(Node):
    # Known distances between tags (in meters)
    HORIZONTAL_DISTANCE = 0.12446  # ID2-ID3, ID4-ID5
    VERTICAL_DISTANCE = 0.05916  # ID2-ID4, ID3-ID5
    DIAGONAL_DISTANCE = 0.13733  # ID2-ID5, ID1-ID4 (calculated from your measurements)

    def __init__(self):
        super().__init__("apriltag_detector")

        self.bridge = CvBridge()
        self.camera_params = None  # will be filled from camera_info.yaml
        self.tag_size = 0.07  # 0.12  # 12 cm

        self.tag_sizes = {
            1: 0.07,  # ID 1 → 70 mm
            2: 0.04,  # ID 2 → 40 mm
            3: 0.04,  # ID 3 → 40 mm
            4: 0.04,  # ID 4 → 40 mm
            5: 0.04,  # ID 5 → 40 mm
        }
        self.show_debug_img = True
        self.show_debug_msg = True

        self.load_camera_info = False

        # Load intrinsics using ROS2 package path (works in Docker and local)
        try:
            pkg_share = get_package_share_directory("go2_auto_dock")
            yaml_path = os.path.join(
                pkg_share, "config", "unitree_go2_cam_indiana", "camera_info.yaml"
            )
            self.load_camera_intrinsics(yaml_path)
        except Exception as e:
            self.get_logger().error(f"Failed to load camera intrinsics: {e}")
            raise

        # Subscriptions
        self.create_subscription(Image, "/camera/image_raw", self.image_callback, 10)

        # Publisher
        self.pose_pub = self.create_publisher(PoseStamped, "/apriltag_pose", 10)
        self.relative_pub = self.create_publisher(
            Float32MultiArray, "/apriltag_relative", 10
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        # AprilTag detector with tuned params
        self.detector = apriltag.Detector(
            options=apriltag.DetectorOptions(
                families="tag36h11",
                nthreads=2,
                quad_decimate=1.0,  # 1.0 = no downsampling (better accuracy)
                refine_edges=True,
                debug=0,
            )
        )

        # For computing callback rate
        self.last_callback_time = None
        self.current_hz = 0.0
        self.bearing_angle = 0.0
        self.bearing_angle_deg_high_accurate = 0.0  # High accuracy bearing angle

    def load_camera_intrinsics(self, yaml_file):
        """Load camera intrinsics from YAML file"""
        if not os.path.exists(yaml_file):
            raise FileNotFoundError(f"Camera info file not found: {yaml_file}")

        with open(yaml_file, "r") as f:
            calib = yaml.safe_load(f)

        fx = calib["camera_matrix"]["data"][0]
        fy = calib["camera_matrix"]["data"][4]
        cx = calib["camera_matrix"]["data"][2]
        cy = calib["camera_matrix"]["data"][5]

        self.camera_params = [fx, fy, cx, cy]
        self.camera_matrix = np.array(
            [[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32
        )

        self.get_logger().info(f"Loaded intrinsics from {yaml_file}")
        self.get_logger().info(
            f"Camera parameters: fx={fx:.2f}, fy={fy:.2f}, cx={cx:.2f}, cy={cy:.2f}"
        )

    def estimate_pose_from_corners(self, corners, tag_size):
        """
        Estimate pose using OpenCV's solvePnP
        """
        # Define 3D corners of the tag (in tag coordinate system)
        tag_corners_3d = np.array(
            [
                [-tag_size / 2, -tag_size / 2, 0],
                [tag_size / 2, -tag_size / 2, 0],
                [tag_size / 2, tag_size / 2, 0],
                [-tag_size / 2, tag_size / 2, 0],
            ],
            dtype=np.float32,
        )

        # Solve PnP
        success, rvec, tvec = cv2.solvePnP(
            tag_corners_3d,
            corners.astype(np.float32),
            self.camera_matrix,
            None,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )

        if success:
            # Convert rotation vector to matrix
            R, _ = cv2.Rodrigues(rvec)
            return R, tvec.reshape(3, 1)
        else:
            return None, None

    def image_callback(self, msg: Image):
        if self.camera_params is None:
            return  # wait until camera_info arrives

        # Convert ROS2 Image → OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")

        # Compute Hz
        now = time.time()
        if self.last_callback_time is not None:
            dt = now - self.last_callback_time
            if dt > 0:
                self.current_hz = 1.0 / dt
        self.last_callback_time = now

        # Draw & publish detections
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

        # Default: no tag seen
        x_right = y_up = z_forward = yaw_error = 0.0
        see_target = 0.0
        see_angle = 0.0

        # Dictionary to store detection with pose information
        tag_dict = {}

        # Detect all tags
        detections = self.detector.detect(frame)

        # Estimate pose for each detection based on tag size
        for det in detections:
            tag_id = det.tag_id
            if tag_id in self.tag_sizes:
                tag_size = self.tag_sizes[tag_id]

                try:
                    # Try using the apriltag detection_pose first
                    result = self.detector.detection_pose(
                        det, self.camera_params, tag_size
                    )

                    if isinstance(result, tuple) and len(result) == 3:
                        # Extract the 4x4 transformation matrix
                        T_matrix = result[0]
                        pose_err1 = result[1]
                        pose_err2 = result[2]

                        # Extract rotation (upper-left 3x3) and translation (upper-right 3x1)
                        pose_R = T_matrix[:3, :3]  # 3x3 rotation matrix
                        pose_t = T_matrix[:3, 3].reshape(3, 1)  # 3x1 translation vector

                        # Store detection with pose information
                        tag_dict[tag_id] = {
                            "detection": det,
                            "pose_R": pose_R,
                            "pose_t": pose_t,
                            "pose_err": pose_err2,  # Use the second error metric
                        }

                        if self.show_debug_msg:
                            self.get_logger().debug(
                                f"Tag {tag_id}: R shape={pose_R.shape}, t shape={pose_t.shape}"
                            )
                    else:
                        # Fallback to OpenCV if format is unexpected
                        pose_R, pose_t = self.estimate_pose_from_corners(
                            det.corners, tag_size
                        )

                        if pose_R is not None and pose_t is not None:
                            tag_dict[tag_id] = {
                                "detection": det,
                                "pose_R": pose_R,
                                "pose_t": pose_t,
                                "pose_err": 0,
                            }
                            if self.show_debug_msg:
                                self.get_logger().info(
                                    f"Used OpenCV fallback for tag {tag_id}"
                                )

                except Exception as e:
                    # Fallback to OpenCV on any error
                    pose_R, pose_t = self.estimate_pose_from_corners(
                        det.corners, tag_size
                    )

                    if pose_R is not None and pose_t is not None:
                        tag_dict[tag_id] = {
                            "detection": det,
                            "pose_R": pose_R,
                            "pose_t": pose_t,
                            "pose_err": 0,
                        }
                        if self.show_debug_msg:
                            self.get_logger().warning(
                                f"Error with apriltag pose for tag {tag_id}: {e}, using OpenCV"
                            )

        # Draw detected tags if debug mode
        if self.show_debug_img:
            for tag_id, tag_info in tag_dict.items():
                det = tag_info["detection"]
                for i in range(4):
                    pt1 = tuple(det.corners[i - 1, :].astype(int))
                    pt2 = tuple(det.corners[i, :].astype(int))
                    cv2.line(frame_bgr, pt1, pt2, (0, 255, 0), 2)
                cv2.circle(frame_bgr, tuple(det.center.astype(int)), 5, (0, 0, 255), -1)
                cv2.putText(
                    frame_bgr,
                    f"ID:{det.tag_id}",
                    (int(det.center[0]), int(det.center[1]) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 255),
                    2,
                )
            cv2.imshow("AprilTag Detection", frame_bgr)
            cv2.waitKey(1)

        if 1 in tag_dict:
            tag_info = tag_dict[1]  # Get the dictionary with detection and pose info
            det = tag_info["detection"]
            pose_R = tag_info["pose_R"]
            pose_t = tag_info["pose_t"]

            see_target = 1.0
            # Pose message
            pose = PoseStamped()
            pose.header = msg.header
            pose.header.frame_id = "camera"

            pose.pose.position.x = float(pose_t[0][0])  # left/right (m)
            pose.pose.position.y = float(pose_t[1][0])  # vertical
            pose.pose.position.z = float(pose_t[2][0])  # forward (m)

            dx = pose.pose.position.x
            dy = pose.pose.position.y
            dz = pose.pose.position.z

            R = pose_R
            q = self.rotation_matrix_to_quaternion(R)
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]

            self.pose_pub.publish(pose)

            yaw_error = np.arctan2(dx, dz)  # radians

            # Debug log  [Relative Yaw in camera frame]
            if self.show_debug_msg:
                self.get_logger().info(
                    f"[Camera] Tag {det.tag_id} at "
                    f"x={pose.pose.position.x:.2f} m, "
                    f"y={pose.pose.position.y:.2f} m, "
                    f"z={pose.pose.position.z:.2f} m, "
                    f"yaw={np.degrees(yaw_error):.1f} deg, "
                )

            # Original tag-to-camera translation
            t = np.array([dx, dy, dz]).reshape(3, 1)  # dx, dy, dz from detection
            R = pose_R  # 3x3 rotation matrix

            # Invert transform to get camera in tag frame
            R_inv = R.T
            t_camera_in_tag = -R_inv @ t  # 3x1 vector

            # Extract components
            x_right = float(t_camera_in_tag[0])  # right of tag
            y_up = float(t_camera_in_tag[1])  # up relative to tag
            z_forward = float(t_camera_in_tag[2])  # forward from tag

            # Compute yaw error (rotation needed to align camera with tag)
            yaw_error = np.arctan2(x_right, z_forward)  # radians

            # Debug print for PID, global coordination yaw
            if self.show_debug_msg:
                self.get_logger().info(
                    f"[Global] Tag {det.tag_id}: "
                    f"right={x_right:.2f} m, up={y_up:.2f} m, forward={z_forward:.2f} m, "
                    f"yaw_error={np.degrees(yaw_error):.1f} deg"
                )

            # --- Broadcast TF for this tag ---
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "camera"
            t.child_frame_id = f"apriltag_{det.tag_id}"

            t.transform.translation.x = dx
            t.transform.translation.y = dy
            t.transform.translation.z = dz
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]

            self.tf_broadcaster.sendTransform(t)

        bearing_result = self.calculate_improved_bearing_angle(tag_dict)

        if bearing_result[0] is not None:
            self.bearing_angle, method_used, valid_bearings = bearing_result
            # Check for high accuracy bearing angle update
            self.check_and_update_high_accuracy_bearing(valid_bearings)
            see_angle = 1.0  # mean see the angle

            if self.show_debug_msg:
                self.get_logger().info(
                    f"[Bearing] {method_used} → {self.bearing_angle:.1f}°"
                )
                for bearing in valid_bearings:
                    self.get_logger().info(
                        f"  {bearing['pair']} ({bearing['type']}): {bearing['angle']:.1f}° "
                        f"(conf: {bearing['confidence']:.2f}, dist_err: {bearing['distance_error']:.3f}, v12: {bearing['v12']}, wall_forward_after_cross:{bearing['wall_forward_after_cross']})"
                    )

        # --- Publish relative vector + yaw_error + Hz + target flag ---
        msg_rel = Float32MultiArray()
        msg_rel.data = [
            x_right,
            y_up,
            z_forward,
            yaw_error,
            self.current_hz,
            see_target,
            self.bearing_angle_deg_high_accurate,
            see_angle,
        ]
        self.relative_pub.publish(msg_rel)

    def rotation_matrix_to_quaternion(self, R):
        qw = np.sqrt(1 + np.trace(R)) / 2
        qx = (R[2, 1] - R[1, 2]) / (4 * qw)
        qy = (R[0, 2] - R[2, 0]) / (4 * qw)
        qz = (R[1, 0] - R[0, 1]) / (4 * qw)
        return (qx, qy, qz, qw)

    def calculate_improved_bearing_angle(self, tag_dict):
        """
        Calculate bearing angle using multiple tag pairs for improved accuracy
        """

        # Define all possible tag pairs and their known distances
        tag_pairs = [
            # Horizontal pairs
            (2, 3, self.HORIZONTAL_DISTANCE, "horizontal"),
            (4, 5, self.HORIZONTAL_DISTANCE, "horizontal"),
            # Diagonal pairs
            (2, 5, self.DIAGONAL_DISTANCE, "diagonal"),
            (4, 3, self.DIAGONAL_DISTANCE, "diagonal"),
        ]

        valid_bearings = []

        for tag_id1, tag_id2, known_distance, pair_type in tag_pairs:
            if tag_id1 in tag_dict and tag_id2 in tag_dict:
                # Extract pose information from the stored dictionaries
                tag1_info = tag_dict[tag_id1]
                tag2_info = tag_dict[tag_id2]

                det1 = tag1_info["detection"]
                det2 = tag2_info["detection"]
                pose_t1 = tag1_info["pose_t"]
                pose_t2 = tag2_info["pose_t"]

                # Extract 3D positions
                p1 = np.array([pose_t1[0][0], pose_t1[1][0], pose_t1[2][0]])
                p2 = np.array([pose_t2[0][0], pose_t2[1][0], pose_t2[2][0]])

                # Vector from tag1 → tag2
                v12 = p2 - p1

                # Scale to exact known distance
                v12_norm = np.linalg.norm(v12)
                if v12_norm > 0:
                    v12 = v12 * (known_distance / v12_norm)

                    # Calculate surface normal for bearing
                    if pair_type == "horizontal":
                        # For horizontal pairs, normal is perpendicular to horizontal vector
                        wall_forward = np.cross(v12, np.array([0, 1, 0]))
                    else:
                        # For diagonal pairs, estimate normal from the diagonal direction
                        wall_forward = np.cross(v12, np.array([0, 1, 0]))

                    wall_forward_after_cross = wall_forward
                    # Normalize the wall forward vector
                    wall_forward_norm = np.linalg.norm(wall_forward)
                    if wall_forward_norm > 0:
                        wall_forward = wall_forward / wall_forward_norm

                        # Calculate bearing angle using atan2 for proper quadrant handling
                        bearing_angle_radian = np.arctan2(
                            wall_forward[0], wall_forward[2]
                        )
                        bearing_angle_deg = np.degrees(bearing_angle_radian)

                        # Normalize to [-180, 180] range
                        if bearing_angle_deg > 180:
                            bearing_angle_deg -= 360
                        elif bearing_angle_deg < -180:
                            bearing_angle_deg += 360

                        # Calculate confidence based on multiple factors
                        detection_confidence = 1.0
                        distance_error = abs(v12_norm - known_distance) / known_distance
                        distance_confidence = max(0.1, 1.0 - distance_error)
                        baseline_confidence = min(2.0, known_distance / 0.05)

                        type_confidence = {
                            "horizontal": 1.0,
                            "vertical": 0.8,
                            "diagonal": 0.9,
                        }[pair_type]

                        total_confidence = (
                            detection_confidence
                            * distance_confidence
                            * baseline_confidence
                            * type_confidence
                        )

                        valid_bearings.append(
                            {
                                "angle": bearing_angle_deg,
                                "pair": f"ID{tag_id1}-ID{tag_id2}",
                                "type": pair_type,
                                "confidence": total_confidence,
                                "distance_error": distance_error,
                                "measured_distance": v12_norm,
                                "v12": v12,
                                "wall_forward_after_cross": wall_forward_after_cross,
                            }
                        )

        if not valid_bearings:
            return None, "No valid tag pairs found", []

        # Calculate weighted average bearing
        if len(valid_bearings) == 1:
            final_bearing = valid_bearings[0]["angle"]
            method_used = f"Single pair: {valid_bearings[0]['pair']}"
        else:
            weights = [bearing["confidence"] for bearing in valid_bearings]
            angles = [bearing["angle"] for bearing in valid_bearings]

            # Handle angle wrapping for averaging (convert to unit vectors)
            x_sum = sum(w * np.cos(np.radians(a)) for w, a in zip(weights, angles))
            y_sum = sum(w * np.sin(np.radians(a)) for w, a in zip(weights, angles))

            final_bearing = np.degrees(np.arctan2(y_sum, x_sum))

            # Normalize to [-180, 180] range
            if final_bearing > 180:
                final_bearing -= 360
            elif final_bearing < -180:
                final_bearing += 360

            method_used = f"Weighted average of {len(valid_bearings)} pairs"

        return final_bearing, method_used, valid_bearings

    def check_and_update_high_accuracy_bearing(self, valid_bearings):
        """
        Update self.bearing_angle_deg_high_accurate only when all four specific
        tag pair estimations (ID2-ID3, ID4-ID5, ID2-ID5, ID4-ID3) are within 3° of their average
        """
        required_pairs = [
            "ID2-ID3",  # horizontal
            "ID4-ID5",  # horizontal
            "ID2-ID5",  # diagonal
            "ID4-ID3",  # diagonal
        ]

        required_angles = {}
        for bearing in valid_bearings:
            if bearing["pair"] in required_pairs:
                required_angles[bearing["pair"]] = bearing["angle"]

        if len(required_angles) == 4:
            angles = list(required_angles.values())
            average_angle = np.mean(angles)

            max_deviation = max(abs(angle - average_angle) for angle in angles)

            if max_deviation <= 3.0:
                self.bearing_angle_deg_high_accurate = average_angle

                if self.show_debug_img:
                    self.get_logger().info(
                        f"[High Accuracy] Updated bearing to {self.bearing_angle_deg_high_accurate:.1f}° "
                        f"(max deviation: {max_deviation:.1f}°)"
                    )
                    for pair, angle in required_angles.items():
                        self.get_logger().info(f"  {pair}: {angle:.1f}°")
            elif self.show_debug_img:
                self.get_logger().info(
                    f"[High Accuracy] Skipping update - max deviation {max_deviation:.1f}° > 3.0°"
                )
        elif self.show_debug_img and len(required_angles) > 0:
            missing_pairs = set(required_pairs) - set(required_angles.keys())
            self.get_logger().info(
                f"[High Accuracy] Missing pairs for update: {missing_pairs}"
            )


def main():
    rclpy.init()
    node = AprilTagNode()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
