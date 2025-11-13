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


# ---------------- path library (0° = +X forward, +Y left) ----------------
def create_straight_line_path_from_angle(angle_deg, length=1.05, num_points=10):
    a = math.radians(angle_deg)
    end_x = length * math.cos(a)  # +X forward
    end_y = length * math.sin(a)  # +Y left
    x_vals = np.linspace(0.0, end_x, num_points)
    y_vals = np.linspace(0.0, end_y, num_points)
    return np.array([x_vals, y_vals])


PATH_ANGLES = [-60, -45, -30, -15, 0, 15, 30, 45, 60, 180]
PATH_LEN = 1.05
PATHS = [create_straight_line_path_from_angle(a, PATH_LEN) for a in PATH_ANGLES]


class OMPath(Node):
    """
    - Obstacles: LaserScan + depth PointCloud (same logic as before).
    - Hazards: subscribe to /traversability/hazard_points2 (or /traversability/hazard_points).
      Hazards only affect the 9 forward rays (0..8). Backward ray (9) ignores hazards.
    - All computations are done in the **robot-centric frame** (0° = +X forward, +Y left).
      We convert incoming hazard points (published in 'laser') into the robot frame by
      rotating them by the lidar mounting yaw.
    - RViz markers are rotated by viz_yaw_deg for display only.
    """

    def __init__(self):
        super().__init__("om_path")

        # Params
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter(
            "depth_obstacle_topic",
            "/camera/realsense2_camera_node/depth/obstacle_point",
        )
        self.declare_parameter("laser_frame", "laser")
        self.declare_parameter(
            "sensor_mounting_angle", 172.0
        )  # deg, +CCW sensor->robot  # <<< same as before
        self.declare_parameter("relevant_distance_min", 0.20)
        self.declare_parameter("half_width_robot", 0.20)
        self.declare_parameter("obstacle_threshold", 0.50)
        self.declare_parameter("hazard_topic_pc2", "/traversability/hazard_points2")
        self.declare_parameter("hazard_topic_pc", "/traversability/hazard_points")

        # Read params
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
        self.mount_deg = float(self.get_parameter("sensor_mounting_angle").value)
        self.min_dist = float(self.get_parameter("relevant_distance_min").value)
        self.half_width = float(self.get_parameter("half_width_robot").value)
        self.obs_thresh = float(self.get_parameter("obstacle_threshold").value)

        self.hazard_topic_pc2 = (
            self.get_parameter("hazard_topic_pc2").get_parameter_value().string_value
        )

        # State
        self.obstacle_cloud: PointCloud | None = None
        self.hazard_xy_robot = np.zeros(
            (0, 2), dtype=np.float32
        )  # <<< CHANGED: store hazards already rotated into ROBOT frame

        # Subs
        self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, 10)
        self.create_subscription(
            PointCloud, self.depth_topic, self.depth_obstacle_cb, 10
        )
        self.create_subscription(
            PointCloud2, self.hazard_topic_pc2, self.hazard_pc2_cb, 10
        )

        # Pubs
        self.paths_pub = self.create_publisher(Paths, "/om/paths", 10)
        self.markers_pub = self.create_publisher(MarkerArray, "/om/paths_markers", 10)

        self.get_logger().info(
            "OMPath (obstacles + hazard PC2, all in robot frame, no OccupancyGrid) started."
        )

    # ---------- helpers ----------
    def _rot_xy(self, x, y, yaw_deg):
        r = math.radians(yaw_deg)
        c, s = math.cos(r), math.sin(r)
        return (x * c - y * s, x * s + y * c)

    def _rot_array_deg(self, arr_xy, yaw_deg):
        if arr_xy.size == 0:
            return arr_xy
        r = math.radians(yaw_deg)
        c, s = math.cos(r), math.sin(r)
        x = arr_xy[:, 0]
        y = arr_xy[:, 1]
        xr = c * x - s * y
        yr = s * x + c * y
        return np.stack([xr, yr], axis=1)

    def _dist_pt_to_segment(self, px, py, x1, y1, x2, y2):
        dx, dy = x2 - x1, y2 - y1
        if dx == 0.0 and dy == 0.0:
            return math.hypot(px - x1, py - y1)
        t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)
        t = max(0.0, min(1.0, t))
        cx, cy = x1 + t * dx, y1 + t * dy
        return math.hypot(px - cx, py - cy)

    # ---------- callbacks ----------
    def depth_obstacle_cb(self, msg: PointCloud):
        self.obstacle_cloud = msg

    def hazard_pc2_cb(self, msg: PointCloud2):
        try:
            arr = pc2.read_points_numpy(msg, field_names=("x", "y"), skip_nans=True)
            arr = np.asarray(arr, dtype=np.float32).reshape(-1, 2)
            # Convert hazards from LASER frame to ROBOT frame by the SAME mounting yaw  # <<< CHANGED
            self.hazard_xy_robot = self._rot_array_deg(arr, self.mount_deg)
        except Exception as e:
            self.get_logger().warn(f"Failed to parse hazard_points2: {e}")

    def scan_cb(self, scan: LaserScan):
        # 1) LaserScan -> points in ROBOT frame (by adding mounting yaw)
        mount = math.radians(self.mount_deg)
        complexes = []
        for i, d_m in enumerate(scan.ranges):
            if not math.isfinite(d_m) or d_m > 5.0 or d_m < self.min_dist:
                continue
            theta = scan.angle_min + i * scan.angle_increment + mount  # sensor -> robot
            #  把极坐标（距离 d 和方位 θ）转成笛卡尔坐标（x,y）
            x = d_m * math.cos(theta)  # +X forward
            y = d_m * math.sin(theta)  # +Y left
            # 把弧度 theta 转成角度，并且归一化到 (-180, 180] eg: 350 degree into -10 degree, left pos right neg
            a_deg = ((math.degrees(theta) + 180.0) % 360.0) - 180.0
            complexes.append([x, y, a_deg, d_m])

        # 2) Add depth obstacles (assumed also already in ROBOT frame; if not, add a TF/rotation here)
        if self.obstacle_cloud and len(self.obstacle_cloud.points) > self.obs_thresh:
            for p in self.obstacle_cloud.points:
                x, y = float(p.x), float(p.y)
                d = math.hypot(x, y)
                a = math.degrees(math.atan2(y, x))
                complexes.append([x, y, a, d])

        if not complexes:
            return

        arr = np.array(complexes)
        arr = arr[arr[:, 2].argsort()]
        X, Y, D = arr[:, 0], arr[:, 1], arr[:, 3]

        N = len(PATH_ANGLES)  # 10

        segments = np.array(
            [
                (PATHS[i][0][0], PATHS[i][1][0], PATHS[i][0][-1], PATHS[i][1][-1])
                for i in range(N)
            ],
            dtype=np.float32,
        )

        blocked_by_obstacle = [False] * N
        blocked_by_hazard = [False] * N

        for x, y, _ in zip(X, Y, D):
            for idx in range(N):
                if idx == 9 and x >= 0.0:
                    continue
                if blocked_by_obstacle[idx]:
                    continue
                x1, y1, x2, y2 = segments[idx]
                if self._dist_pt_to_segment(x, y, x1, y1, x2, y2) <= self.half_width:
                    blocked_by_obstacle[idx] = True

        # 3.2 hazard 命中：只作用于前向 0..8
        if self.hazard_xy_robot.size > 0:
            for hx, hy in self.hazard_xy_robot:
                for idx in range(0, N - 1):  # 0..8
                    if blocked_by_hazard[idx]:
                        continue
                    x1, y1, x2, y2 = segments[idx]
                    if (
                        self._dist_pt_to_segment(hx, hy, x1, y1, x2, y2)
                        <= self.half_width
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
        msg_paths.blocked_by_obstacle = blocked_by_obstacle
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
        ma = MarkerArray()

        wipe = Marker()
        wipe.header.frame_id = frame_id
        wipe.header.stamp = stamp
        wipe.action = Marker.DELETEALL
        ma.markers.append(wipe)

        # Rays
        for idx, path_arr in enumerate(PATHS):
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
                px, py = self._rot_xy(float(x), float(y), -self.mount_deg)
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
            tx, ty = self._rot_xy(ex, ey, -self.mount_deg)
            t.pose.position.x = tx
            t.pose.position.y = ty
            t.pose.position.z = 0.06
            ang_disp = ((PATH_ANGLES[idx] - self.mount_deg + 180.0) % 360.0) - 180.0
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
                px, py = self._rot_xy(float(x), float(y), -self.mount_deg)
                pts.points.append(Point(x=px, y=py, z=0.0))
            ma.markers.append(pts)

        # Hazards (blue) — already in ROBOT frame; rotate only for display
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
                px, py = self._rot_xy(float(x), float(y), -self.mount_deg)
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
