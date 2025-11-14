#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Run:
PYTHONNOUSERSITE=1 python3 /home/openmind/Desktop/wendy-work-station/unitree_go2_ros2_sdk/go2_sdk/go2_sdk/om_path_trave_base.py \
  --ros-args -p hazard_topic_pc2:=/traversability/hazard_points2
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from visualization_msgs.msg import Marker, MarkerArray

from om_api.msg import Paths


def create_straight_line_path_from_angle(angle_degrees, length=1.05, num_points=10):
    """Create a straight line path from origin at specified angle and length.

    Parameters
    ----------
    angle_degrees : float
        Heading in degrees; here 0° ≡ +X forward, +Y left (robot frame).
    length : float
        Path length in meters.
    num_points : int
        Number of samples along the path.

    Returns
    -------
    np.ndarray
        Shape (2, num_points) with x row then y row.
    """
    a = math.radians(angle_degrees)
    end_x = length * math.cos(a)  # +X forward
    end_y = length * math.sin(a)  # +Y left
    x_vals = np.linspace(0.0, end_x, num_points)
    y_vals = np.linspace(0.0, end_y, num_points)
    return np.array([x_vals, y_vals])


# Define 9 straight line paths separated by 15 degrees (+ one reverse at 180°)
path_angles = [-60, -45, -30, -15, 0, 15, 30, 45, 60, 180]  # degrees
path_length = 1.05  # meters
paths = [create_straight_line_path_from_angle(a, path_length) for a in path_angles]


class OMPath(Node):
    """Fuse LaserScan, depth obstacles, and hazard PC2 to choose feasible paths.

    Notes
    -----
    * All computations in robot frame: +X forward, +Y left.
    * Hazards (stairs/holes/edges) only block forward rays (indices 0..8).
    * Reverse ray (index 9) ignores hazards and only checks points behind robot (x<0).
    """

    def __init__(self):
        super().__init__("om_path")

        # === old-style member variables (no params except hazard topic) ===
        self.half_width_robot = 0.20
        self.sensor_mounting_angle = 172.0  # deg, +CCW sensor->robot
        self.relevant_distance_min = 0.20
        self.obstacle_threshold = 0.50  # min #points to consider depth cloud
        self.laser_frame = "laser"  # keep consistent frame used for publishing

        # Only this topic remains parameterized (compatibility with your CLI)
        self.declare_parameter("hazard_topic_pc2", "/traversability/hazard_points2")
        self.hazard_topic_pc2 = (
            self.get_parameter("hazard_topic_pc2").get_parameter_value().string_value
        )

        # State like the old file
        self.scan = None
        self.obstacle: PointCloud | None = None
        self.hazard_xy_robot = np.zeros((0, 2), dtype=np.float32)

        # Subscriptions (names/shape like old file)
        self.scan_info = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 10
        )
        self.obstacle_info = self.create_subscription(
            PointCloud,
            "/camera/realsense2_camera_node/depth/obstacle_point",
            self.obstacle_callback,
            10,
        )
        self.hazard_info = self.create_subscription(
            PointCloud2, self.hazard_topic_pc2, self.hazard_callback_pc2, 10
        )

        # Publisher identical to old file
        self.paths_pub = self.create_publisher(Paths, "/om/paths", 10)
        self.markers_pub = self.create_publisher(MarkerArray, "/om/paths_markers", 10)

        self.get_logger().info("OMPath node started (with hazard PC2 support)")

    def scan_callback(self, msg: LaserScan):
        """Main fuse/select logic presented in the old style (same names/flow)."""
        self.scan = msg

        # Keep angles in radians, do not flip, add mounting in radians
        angles = list(
            np.arange(
                self.scan.angle_min, self.scan.angle_max, self.scan.angle_increment
            )
        )
        angles_final = angles
        data = list(zip(angles_final, self.scan.ranges))

        complexes = []
        mount = math.radians(self.sensor_mounting_angle)

        for angle_rad, d_m in data:
            if not math.isfinite(d_m) or d_m > 5.0 or d_m < self.relevant_distance_min:
                continue

            # sensor -> robot
            theta = angle_rad + mount

            # robot frame: x = d*cos, y = d*sin  (0° = +X forward, +Y left)
            x = d_m * math.cos(theta)
            y = d_m * math.sin(theta)

            # angle for sorting/compatibility: [-180, 180)
            a_deg = ((math.degrees(theta) + 180.0) % 360.0) - 180.0

            complexes.append([x, y, a_deg, d_m])

        # Depth obstacles (already in robot frame)
        if self.obstacle and len(self.obstacle.points) > self.obstacle_threshold:
            for p in self.obstacle.points:
                x, y = float(p.x), float(p.y)
                angle, distance = self.calculate_angle_and_distance(x, y)
                complexes.append([x, y, angle, distance])

        if not complexes:
            return

        array = np.array(complexes)
        sorted_indices = array[:, 2].argsort()
        array = array[sorted_indices]

        X = array[:, 0]
        Y = array[:, 1]
        D = array[:, 3]

        N = len(path_angles)
        possible_paths = np.arange(0, N)  # 0..9
        bad_paths = []  # kept for compatibility/inspection

        # NEW: record blocked path indices instead of boolean arrays
        blocked_by_obstacle_set = set()
        blocked_by_hazard_set = set()

        # Obstacle-based blocking
        for x, y, d in list(zip(X, Y, D)):
            for apath in possible_paths.copy():
                # Reverse ray only considers obstacles behind robot
                if apath == 9 and x >= 0.0:
                    continue

                path_points = paths[apath]
                start_x, start_y = path_points[0][0], path_points[1][0]
                end_x, end_y = path_points[0][-1], path_points[1][-1]
                dist_to_line = self.distance_point_to_line_segment(
                    x, y, start_x, start_y, end_x, end_y
                )
                if dist_to_line < self.half_width_robot:
                    bad_paths.append(apath)
                    blocked_by_obstacle_set.add(int(apath))
                    possible_paths = np.setdiff1d(possible_paths, np.array([apath]))
                    break

        # Hazard-based blocking: only forward rays (0..8)
        if self.hazard_xy_robot.size > 0 and possible_paths.size > 0:
            for hx, hy in self.hazard_xy_robot:
                for apath in possible_paths.copy():
                    if apath == 9:
                        continue
                    path_points = paths[apath]
                    start_x, start_y = path_points[0][0], path_points[1][0]
                    end_x, end_y = path_points[0][-1], path_points[1][-1]
                    dist_to_line = self.distance_point_to_line_segment(
                        hx, hy, start_x, start_y, end_x, end_y
                    )
                    if dist_to_line < self.half_width_robot:
                        bad_paths.append(apath)
                        blocked_by_hazard_set.add(int(apath))
                        possible_paths = np.setdiff1d(possible_paths, np.array([apath]))
                        break

        # Convert sets to sorted index lists
        blocked_by_obstacle_idx = [int(i) for i in sorted(blocked_by_obstacle_set)]
        blocked_by_hazard_idx = [int(i) for i in sorted(blocked_by_hazard_set)]

        # Publish (same shape) + optional index lists for clarity
        paths_msg = Paths()
        paths_msg.header.stamp = self.get_clock().now().to_msg()
        paths_msg.header.frame_id = self.laser_frame
        paths_msg.paths = [int(i) for i in possible_paths.tolist()]

        # Prefer index-style fields if your msg supports them
        if hasattr(paths_msg, "blocked_by_obstacle_idx"):
            paths_msg.blocked_by_obstacle_idx = blocked_by_obstacle_idx
        if hasattr(paths_msg, "blocked_by_hazard_idx"):
            paths_msg.blocked_by_hazard_idx = blocked_by_hazard_idx

        self.paths_pub.publish(paths_msg)

        # RViz markers (display uses -mounting angle)
        bad_union = blocked_by_obstacle_set | blocked_by_hazard_set
        self._publish_markers(
            possible_paths=set(paths_msg.paths),
            bad_paths=bad_union,
            obstacles_xy=list(zip(X.tolist(), Y.tolist())),
            hazards_xy=self.hazard_xy_robot.tolist(),
            frame_id=self.laser_frame,
            stamp=paths_msg.header.stamp,
        )

    def obstacle_callback(self, msg: PointCloud):
        """Store depth-obstacle PointCloud (assumed already in robot frame)."""
        self.obstacle = msg

    def _rot_array_deg(self, arr_xy: np.ndarray, yaw_deg: float) -> np.ndarray:
        """Rotate Nx2 [x,y] points by yaw_deg (degrees, +CCW)."""
        if arr_xy.size == 0:
            return arr_xy
        r = math.radians(yaw_deg)
        c, s = math.cos(r), math.sin(r)
        x = arr_xy[:, 0]
        y = arr_xy[:, 1]
        xr = c * x - s * y
        yr = s * x + c * y
        return np.stack([xr, yr], axis=1)

    def hazard_callback_pc2(self, msg: PointCloud2):
        """Read hazard PC2 (x,y), rotate from LASER to robot frame, store as Nx2."""
        try:
            arr = pc2.read_points_numpy(msg, field_names=("x", "y"), skip_nans=True)
            arr = np.asarray(arr, dtype=np.float32).reshape(-1, 2)
            self.hazard_xy_robot = self._rot_array_deg(arr, self.sensor_mounting_angle)
        except Exception as e:
            self.get_logger().warn(f"Failed to parse hazard_points2: {e}")

    def calculate_angle_and_distance(self, world_x, world_y):
        """Return (angle_degrees, distance) from x,y in robot frame."""
        distance = math.sqrt(world_x**2 + world_y**2)
        angle_rad = math.atan2(world_y, world_x)
        angle_degrees = math.degrees(angle_rad)
        return angle_degrees, distance

    def distance_point_to_line_segment(self, px, py, x1, y1, x2, y2):
        """Distance from (px,py) to segment (x1,y1)-(x2,y2)."""
        dx = x2 - x1
        dy = y2 - y1
        if dx == 0 and dy == 0:
            return math.sqrt((px - x1) ** 2 + (py - y1) ** 2)
        t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)
        t = max(0, min(1, t))
        closest_x = x1 + t * dx
        closest_y = y1 + t * dy
        return math.sqrt((px - closest_x) ** 2 + (py - closest_y) ** 2)

    # ---------- marker publishing ----------
    def _publish_markers(
        self, possible_paths, bad_paths, obstacles_xy, hazards_xy, frame_id, stamp
    ):
        """Publish RViz markers for candidate paths, obstacles, and hazards."""
        ma = MarkerArray()

        wipe = Marker()
        wipe.header.frame_id = frame_id
        wipe.header.stamp = stamp
        wipe.action = Marker.DELETEALL
        ma.markers.append(wipe)

        # Rays (colored by availability)
        for idx, path_arr in enumerate(paths):
            line = Marker()
            line.header.frame_id = frame_id
            line.header.stamp = stamp
            line.ns = "candidate_paths"
            line.id = idx
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.scale.x = 0.02

            if idx in possible_paths:
                line.color.r, line.color.g, line.color.b, line.color.a = (
                    0.0,
                    1.0,
                    0.0,
                    1.0,
                )
            elif idx in bad_paths:
                line.color.r, line.color.g, line.color.b, line.color.a = (
                    1.0,
                    0.0,
                    0.0,
                    1.0,
                )
            else:
                line.color.r, line.color.g, line.color.b, line.color.a = (
                    0.6,
                    0.6,
                    0.6,
                    1.0,
                )

            # Display rotation: -sensor_mounting_angle
            rot = -self.sensor_mounting_angle
            for x, y in zip(path_arr[0], path_arr[1]):
                a = math.radians(rot)
                c, s = math.cos(a), math.sin(a)
                px, py = c * float(x) - s * float(y), s * float(x) + c * float(y)
                line.points.append(Point(x=px, y=py, z=0.0))
            ma.markers.append(line)

            # Endpoint labels
            t = Marker()
            t.header.frame_id = frame_id
            t.header.stamp = stamp
            t.ns = "path_labels"
            t.id = 1000 + idx
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.scale.z = 0.08
            t.color.r = t.color.g = t.color.b = 1.0
            t.color.a = 0.9
            ex, ey = float(path_arr[0][-1]), float(path_arr[1][-1])
            a = math.radians(rot)
            c, s = math.cos(a), math.sin(a)
            tx, ty = c * ex - s * ey, s * ex + c * ey
            t.pose.position.x = tx
            t.pose.position.y = ty
            t.pose.position.z = 0.06
            ang_disp = (
                (path_angles[idx] - self.sensor_mounting_angle + 180.0) % 360.0
            ) - 180.0
            t.text = f"{idx} ({int(ang_disp)}°)"
            ma.markers.append(t)

        # Obstacles (orange)
        if obstacles_xy:
            pts = Marker()
            pts.header.frame_id = frame_id
            pts.header.stamp = stamp
            pts.ns = "obstacles"
            pts.id = 2000
            pts.type = Marker.POINTS
            pts.action = Marker.ADD
            pts.scale.x = pts.scale.y = 0.03
            pts.color.r, pts.color.g, pts.color.b, pts.color.a = 1.0, 0.5, 0.0, 0.9
            rot = -self.sensor_mounting_angle
            a = math.radians(rot)
            c, s = math.cos(a), math.sin(a)
            for x, y in obstacles_xy:
                px, py = c * float(x) - s * float(y), s * float(x) + c * float(y)
                pts.points.append(Point(x=px, y=py, z=0.0))
            ma.markers.append(pts)

        # Hazards (blue)
        if hazards_xy:
            h = Marker()
            h.header.frame_id = frame_id
            h.header.stamp = stamp
            h.ns = "hazards"
            h.id = 3000
            h.type = Marker.POINTS
            h.action = Marker.ADD
            h.scale.x = h.scale.y = 0.035
            h.color.r, h.color.g, h.color.b, h.color.a = 0.2, 0.5, 0.9, 0.9
            rot = -self.sensor_mounting_angle
            a = math.radians(rot)
            c, s = math.cos(a), math.sin(a)
            for x, y in hazards_xy:
                px, py = c * float(x) - s * float(y), s * float(x) + c * float(y)
                h.points.append(Point(x=px, y=py, z=0.0))
            ma.markers.append(h)

        self.markers_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = OMPath()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
