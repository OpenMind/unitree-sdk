#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
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
    """
    Create a straight-line path in the robot frame.

    Parameters
    ----------
    angle_degrees : float
        Path heading in degrees where 0° is +X (forward) and +Y is left.
    length : float, optional
        Total path length in meters (default 1.05).
    num_points : int, optional
        Number of points sampled along the path (default 10).

    Returns
    -------
    np.ndarray
        A (2, num_points) array where row 0 is x values and row 1 is y values.
    """
    a = math.radians(angle_degrees)
    end_x = length * math.cos(a)  # +X forward
    end_y = length * math.sin(a)  # +Y left
    x_vals = np.linspace(0.0, end_x, num_points)
    y_vals = np.linspace(0.0, end_y, num_points)
    return np.array([x_vals, y_vals])


path_angles = [-60, -45, -30, -15, 0, 15, 30, 45, 60, 180]
path_length = 1.05
paths = [create_straight_line_path_from_angle(a, path_length) for a in path_angles]


class OMPath(Node):
    """
    ROS 2 node that fuses LaserScan, depth obstacles, and hazard PointCloud2 to
    select feasible straight-line paths and publish RViz markers.

    - All computation is in robot frame (+X forward, +Y left).
    - Hazards affect only forward rays (indices 0..8); the backward ray (index 9) ignores hazards.
    - RViz visualization is rotated by -sensor_mounting_angle for display only.
    """

    def __init__(self):

        super().__init__("om_path")

        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter(
            "depth_obstacle_topic",
            "/camera/realsense2_camera_node/depth/obstacle_point",
        )
        self.declare_parameter("laser_frame", "laser")
        self.declare_parameter("sensor_mounting_angle", 172.0)
        self.declare_parameter("relevant_distance_min", 0.20)
        self.declare_parameter("half_width_robot", 0.20)
        self.declare_parameter("obstacle_threshold", 0.50)
        self.declare_parameter("hazard_topic_pc2", "/traversability/hazard_points2")

        self.scan_topic = (
            self.get_parameter("scan_topic").get_parameter_value().string_value
        )
        self.depth_topic = (
            self.get_parameter("depth_obstacle_topic")
            .get_parameter_value()
            .string_value
        )
        self.laser_frame = (
            self.get_parameter("laser_frame").get_parameter_value().string_value
        )

        self.sensor_mounting_angle = float(
            self.get_parameter("sensor_mounting_angle").value
        )
        self.relevant_distance_min = float(
            self.get_parameter("relevant_distance_min").value
        )
        self.half_width_robot = float(self.get_parameter("half_width_robot").value)
        self.obstacle_threshold = float(self.get_parameter("obstacle_threshold").value)

        self.hazard_topic_pc2 = (
            self.get_parameter("hazard_topic_pc2").get_parameter_value().string_value
        )

        self.obstacle: PointCloud | None = None
        self.hazard_xy_robot = np.zeros((0, 2), dtype=np.float32)

        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
        self.create_subscription(
            PointCloud, self.depth_topic, self.obstacle_callback, 10
        )
        self.create_subscription(
            PointCloud2, self.hazard_topic_pc2, self.hazard_callback_pc2, 10
        )

        self.paths_pub = self.create_publisher(Paths, "/om/paths", 10)
        self.markers_pub = self.create_publisher(MarkerArray, "/om/paths_markers", 10)

        self.get_logger().info("OMPath (obstacles + hazard PC2, robot-frame) started.")

    def _rot_xy(self, x, y, yaw_deg):
        """
        Rotate a 2D point (x, y) by yaw_deg in degrees.

        Parameters
        ----------
        x : float
            X coordinate in the source frame.
        y : float
            Y coordinate in the source frame.
        yaw_deg : float
            Rotation angle in degrees, +CCW.

        Returns
        -------
        tuple[float, float]
            Rotated (x', y') coordinates.
        """
        r = math.radians(yaw_deg)
        c, s = math.cos(r), math.sin(r)
        return (x * c - y * s, x * s + y * c)

    def _rot_array_deg(self, arr_xy, yaw_deg):
        """
        Rotate an Nx2 array of 2D points by yaw_deg in degrees.

        Parameters
        ----------
        arr_xy : np.ndarray
            Array with shape (N, 2) containing [x, y] rows.
        yaw_deg : float
            Rotation angle in degrees, +CCW.

        Returns
        -------
        np.ndarray
            Rotated array of shape (N, 2).
        """
        if arr_xy.size == 0:
            return arr_xy
        r = math.radians(yaw_deg)
        c, s = math.cos(r), math.sin(r)
        x = arr_xy[:, 0]
        y = arr_xy[:, 1]
        xr = c * x - s * y
        yr = s * x + c * y
        return np.stack([xr, yr], axis=1)

    def distance_point_to_line_segment(self, px, py, x1, y1, x2, y2):
        """
        Compute the distance from a point to a line segment.

        Parameters
        ----------
        px, py : float
            Point coordinates.
        x1, y1 : float
            Segment start coordinates.
        x2, y2 : float
            Segment end coordinates.

        Returns
        -------
        float
            Euclidean distance from (px, py) to the closest point on the segment.
        """
        dx, dy = x2 - x1, y2 - y1
        if dx == 0.0 and dy == 0.0:
            return math.hypot(px - x1, py - y1)
        t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)
        t = max(0.0, min(1.0, t))
        cx, cy = x1 + t * dx, y1 + t * dy
        return math.hypot(px - cx, py - cy)

    def obstacle_callback(self, msg: PointCloud):
        """
        Receive and store the depth-obstacle PointCloud.

        Parameters
        ----------
        msg : sensor_msgs.msg.PointCloud
            Obstacle cloud (assumed already in the robot frame).

        Returns
        -------
        None
        """
        self.obstacle = msg

    def hazard_callback_pc2(self, msg: PointCloud2):
        """
        Receive hazard points (PointCloud2) and rotate them into the robot frame.

        Parameters
        ----------
        msg : sensor_msgs.msg.PointCloud2
            Hazard point cloud published in the LASER frame (x, y fields used).

        Returns
        -------
        None
        """
        try:
            arr = pc2.read_points_numpy(msg, field_names=("x", "y"), skip_nans=True)
            arr = np.asarray(arr, dtype=np.float32).reshape(-1, 2)
            # hazard 来自 laser，需要转到“机器人系”：用相同的安装角（+CCW）
            self.hazard_xy_robot = self._rot_array_deg(arr, self.sensor_mounting_angle)
        except Exception as e:
            self.get_logger().warn(f"Failed to parse hazard_points2: {e}")

    def scan_callback(self, scan: LaserScan):
        """
        Fuse LaserScan, depth obstacles, and hazards; select feasible paths; publish markers.

        Parameters
        ----------
        scan : sensor_msgs.msg.LaserScan
            Laser scan in sensor frame (will be rotated into robot frame by sensor_mounting_angle).

        Returns
        -------
        None
        """

        mount = math.radians(self.sensor_mounting_angle)
        complexes = []
        for i, d_m in enumerate(scan.ranges):
            if not math.isfinite(d_m) or d_m > 5.0 or d_m < self.relevant_distance_min:
                continue
            theta = scan.angle_min + i * scan.angle_increment + mount  # sensor -> robot
            x = d_m * math.cos(theta)  # +X forward
            y = d_m * math.sin(theta)  # +Y left
            a_deg = ((math.degrees(theta) + 180.0) % 360.0) - 180.0
            complexes.append([x, y, a_deg, d_m])

        if self.obstacle and len(self.obstacle.points) > self.obstacle_threshold:
            for p in self.obstacle.points:
                x, y = float(p.x), float(p.y)
                d = math.hypot(x, y)
                a = math.degrees(math.atan2(y, x))
                complexes.append([x, y, a, d])

        if not complexes:
            return

        arr = np.array(complexes)
        arr = arr[arr[:, 2].argsort()]
        X, Y, D = arr[:, 0], arr[:, 1], arr[:, 3]

        N = len(path_angles)  # 10
        segments = np.array(
            [
                (paths[i][0][0], paths[i][1][0], paths[i][0][-1], paths[i][1][-1])
                for i in range(N)
            ],
            dtype=np.float32,
        )

        blocked_by_obstacle = [False] * N
        blocked_by_hazard = [False] * N

        for x, y, _ in zip(X, Y, D):
            for idx in range(N):
                # 后退（idx=9）只看后侧
                if idx == 9 and x >= 0.0:
                    continue
                if blocked_by_obstacle[idx]:
                    continue
                x1, y1, x2, y2 = segments[idx]
                if (
                    self.distance_point_to_line_segment(x, y, x1, y1, x2, y2)
                    <= self.half_width_robot
                ):
                    blocked_by_obstacle[idx] = True

        if self.hazard_xy_robot.size > 0:
            for hx, hy in self.hazard_xy_robot:
                for idx in range(0, N - 1):  # 0..8
                    if blocked_by_hazard[idx]:
                        continue
                    x1, y1, x2, y2 = segments[idx]
                    if (
                        self.distance_point_to_line_segment(hx, hy, x1, y1, x2, y2)
                        <= self.half_width_robot
                    ):
                        blocked_by_hazard[idx] = True

        possible = [
            i for i in range(N) if not (blocked_by_obstacle[i] or blocked_by_hazard[i])
        ]

        stamp = self.get_clock().now().to_msg()
        msg_paths = Paths()
        msg_paths.header.stamp = stamp
        msg_paths.header.frame_id = self.laser_frame
        msg_paths.paths = possible

        if hasattr(msg_paths, "blocked_by_obstacle"):
            msg_paths.blocked_by_obstacle = blocked_by_obstacle
        if hasattr(msg_paths, "blocked_by_hazard"):
            msg_paths.blocked_by_hazard = blocked_by_hazard

        self.paths_pub.publish(msg_paths)

        bad_union = {
            i for i in range(N) if blocked_by_obstacle[i] or blocked_by_hazard[i]
        }
        self._publish_markers(
            possible_paths=set(possible),
            bad_paths=bad_union,
            obstacles_xy=list(zip(X.tolist(), Y.tolist())),
            hazards_xy=self.hazard_xy_robot.tolist(),
            frame_id=self.laser_frame,
            stamp=stamp,
        )

    # ---------- marker publishing ----------
    def _publish_markers(
        self, possible_paths, bad_paths, obstacles_xy, hazards_xy, frame_id, stamp
    ):
        """
        Publish RViz markers for candidate paths, obstacles, and hazards.

        Parameters
        ----------
        possible_paths : set[int]
            Indices of feasible paths.
        bad_paths : set[int]
            Indices of blocked paths (by obstacle and/or hazard).
        obstacles_xy : list[tuple[float, float]]
            Obstacle points (x, y) in robot frame.
        hazards_xy : list[list[float]] | list[tuple[float, float]]
            Hazard points (x, y) in robot frame.
        frame_id : str
            TF frame to attach the markers to (usually 'laser').
        stamp : builtin_interfaces.msg.Time
            Timestamp for the markers.
        """
        ma = MarkerArray()

        wipe = Marker()
        wipe.header.frame_id = frame_id
        wipe.header.stamp = stamp
        wipe.action = Marker.DELETEALL
        ma.markers.append(wipe)

        # Rays
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

            for x, y in zip(path_arr[0], path_arr[1]):
                px, py = self._rot_xy(float(x), float(y), -self.sensor_mounting_angle)
                line.points.append(Point(x=px, y=py, z=0.0))
            ma.markers.append(line)

            # labels
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
            tx, ty = self._rot_xy(ex, ey, -self.sensor_mounting_angle)
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
            for x, y in obstacles_xy:
                px, py = self._rot_xy(float(x), float(y), -self.sensor_mounting_angle)
                pts.points.append(Point(x=px, y=py, z=0.0))
            ma.markers.append(pts)

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
            for x, y in hazards_xy:
                px, py = self._rot_xy(float(x), float(y), -self.sensor_mounting_angle)
                h.points.append(Point(x=px, y=py, z=0.0))
            ma.markers.append(h)

        self.markers_pub.publish(ma)


def main(args=None):
    """
    Entry point: initialize rclpy, spin the OMPath node, and shut down.
    """
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
