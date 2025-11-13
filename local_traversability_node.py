#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Run me like this (single terminal):

PYTHONNOUSERSITE=1 python3 local_traversability_node.py --ros-args \
  -p target_frame:=odom \
  -p grid_res:=0.05 -p grid_size:=5.0 \
  -p max_range:=2.0 -p half_width:=2.5 \
  -p slope_deg_thresh:=10.0 -p downhill_deg_thresh:=8.0 \
  -p step_thresh_m:=0.12 -p hole_thresh_m:=0.15 \
  -p down_step_thresh_m:=0.10 -p down_step_k_max:=6 \
  -p slope_baseline_cells:=3 \
  -p gradual_drop_window_m:=0.30 -p gradual_drop_thresh_m:=0.10 \
  -p unknown_ahead_window_m:=0.15 -p unknown_ahead_frac_thresh:=0.85 \
  -p min_block_window_m:=0.20 \
  -p cloud_topic:=/camera/depth/points
"""

import atexit
import math
import os
import signal
import subprocess
import sys

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import Point32
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from scipy.ndimage import (
    binary_dilation,
    binary_opening,
    label,
    maximum_filter,
    minimum_filter,
)
from sensor_msgs.msg import PointCloud, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header

# ---------------- helper: manage background ROS 2 helpers ----------------

_procs = []


def _kill_process_tree():
    """Terminate any helper processes we spawned."""
    for p in _procs:
        try:
            if p.poll() is None:
                if hasattr(os, "getpgid"):
                    os.killpg(os.getpgid(p.pid), signal.SIGTERM)
                else:
                    p.terminate()
        except Exception:
            pass


atexit.register(_kill_process_tree)


def _launch_ros2_helpers(logger=None):
    """
    Launch:
      1) depth_image_proc point_cloud_xyz_node
      2) tf2_ros static_transform_publisher

    Hard-coded exactly as requested; no extra parameters are declared.
    """
    cmds = [
        [
            "ros2",
            "run",
            "depth_image_proc",
            "point_cloud_xyz_node",
            "--ros-args",
            "-r",
            "image_rect:=/camera/realsense2_camera_node/depth/image_rect_raw",
            "-r",
            "camera_info:=/camera/realsense2_camera_node/depth/camera_info",
            "-r",
            "points:=/camera/depth/points",
        ],
        [
            "ros2",
            "run",
            "tf2_ros",
            "static_transform_publisher",
            "--x",
            "0.2",
            "--y",
            "0",
            "--z",
            "0.30",
            "--roll",
            "0",
            "--pitch",
            "0.5934119457",
            "--yaw",
            "0",
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "camera_link",
        ],
    ]

    for cmd in cmds:
        try:
            # start each in its own process group so we can kill group on exit
            p = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid if hasattr(os, "setsid") else None,
            )
            _procs.append(p)
            if logger:
                logger.info(f"Started helper: {' '.join(cmd)} (pid={p.pid})")
        except FileNotFoundError:
            if logger:
                logger.error(
                    f"Failed to start helper: {' '.join(cmd)}. "
                    "Make sure ROS 2 is sourced and the package is installed."
                )
            raise
        except Exception as e:
            if logger:
                logger.error(f"Error starting helper: {' '.join(cmd)} -> {e}")
            raise


# ---------------- helpers (math/TF) ----------------


def tf_to_matrix(tf):
    q = tf.transform.rotation
    t = tf.transform.translation
    qw, qx, qy, qz = q.w, q.x, q.y, q.z
    R = np.array(
        [
            [
                1 - 2 * (qy * qy + qz * qz),
                2 * (qx * qy - qz * qw),
                2 * (qx * qz + qy * qw),
            ],
            [
                2 * (qx * qy + qz * qw),
                1 - 2 * (qx * qx + qz * qz),
                2 * (qy * qz - qx * qw),
            ],
            [
                2 * (qx * qz - qy * qw),
                2 * (qy * qz + qx * qw),
                1 - 2 * (qx * qx + qy * qy),
            ],
        ],
        dtype=np.float32,
    )
    T = np.eye(4, dtype=np.float32)
    T[:3, :3] = R
    T[:3, 3] = [t.x, t.y, t.z]
    return T


def quat2euler(x, y, z, w):
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def euler_to_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=np.float32)
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=np.float32)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=np.float32)
    return (Rz @ Ry @ Rx).astype(np.float32)


# ---------------- node ----------------
class LocalTraversability(Node):
    def __init__(self):
        super().__init__("local_traversability")
        # ----- parameters (same defaults you used previously) -----
        self.declare_parameter("cloud_type", "pc2")
        self.declare_parameter("cloud_topic", "/camera/depth/points")
        self.declare_parameter("target_frame", "odom")
        self.declare_parameter("grid_res", 0.05)
        self.declare_parameter("grid_size", 5.0)
        self.declare_parameter("max_range", 2.0)
        self.declare_parameter("half_width", 2.5)
        self.declare_parameter("min_z", -1.0)
        self.declare_parameter("max_z", 2.0)
        self.declare_parameter("slope_deg_thresh", 10.0)
        self.declare_parameter("downhill_deg_thresh", 8.0)
        self.declare_parameter("step_thresh_m", 0.12)
        self.declare_parameter("hole_thresh_m", 0.15)
        self.declare_parameter("down_step_thresh_m", 0.10)
        self.declare_parameter("slope_baseline_cells", 3)
        self.declare_parameter("down_step_k_max", 6)
        self.declare_parameter("gradual_drop_window_m", 0.30)
        self.declare_parameter("gradual_drop_thresh_m", 0.10)
        self.declare_parameter("unknown_ahead_window_m", 0.15)
        self.declare_parameter("unknown_ahead_frac_thresh", 0.85)
        self.declare_parameter("min_block_window_m", 0.20)
        self.declare_parameter("inflate_radius_m", 0.15)

        # NEW: publish hazards topics/frames (as in your original file)
        self.declare_parameter("hazard_output_topic", "/traversability/hazard_points")
        self.declare_parameter("hazard_output_frame", "laser")
        self.declare_parameter("hazard_include_unknown", False)
        self.declare_parameter("hazard_stride", 1)
        self.declare_parameter(
            "hazard_output_topic_pc2", "/traversability/hazard_points2"
        )

        # ----- read params -----
        self.cloud_topic = (
            self.get_parameter("cloud_topic").get_parameter_value().string_value
        )
        self.target_frame = (
            self.get_parameter("target_frame").get_parameter_value().string_value
        )
        self.grid_res = float(self.get_parameter("grid_res").value)
        self.grid_size = float(self.get_parameter("grid_size").value)
        self.max_range = float(self.get_parameter("max_range").value)
        self.half_width = float(self.get_parameter("half_width").value)
        self.min_z = float(self.get_parameter("min_z").value)
        self.max_z = float(self.get_parameter("max_z").value)
        self.slope_deg_thresh = float(self.get_parameter("slope_deg_thresh").value)
        self.downhill_deg_thresh = float(
            self.get_parameter("downhill_deg_thresh").value
        )
        self.step_thresh_m = float(self.get_parameter("step_thresh_m").value)
        self.hole_thresh_m = float(self.get_parameter("hole_thresh_m").value)
        self.down_step_thresh_m = float(self.get_parameter("down_step_thresh_m").value)
        self.slope_baseline_cells = int(
            self.get_parameter("slope_baseline_cells").value
        )
        self.down_step_k_max = int(self.get_parameter("down_step_k_max").value)
        self.gradual_drop_window_m = float(
            self.get_parameter("gradual_drop_window_m").value
        )
        self.gradual_drop_thresh_m = float(
            self.get_parameter("gradual_drop_thresh_m").value
        )
        self.unknown_ahead_window_m = float(
            self.get_parameter("unknown_ahead_window_m").value
        )
        self.unknown_ahead_frac_thresh = float(
            self.get_parameter("unknown_ahead_frac_thresh").value
        )
        self.min_block_window_m = float(self.get_parameter("min_block_window_m").value)
        self.inflate_radius_m = float(self.get_parameter("inflate_radius_m").value)

        self.hazard_output_topic = (
            self.get_parameter("hazard_output_topic").get_parameter_value().string_value
        )
        self.hazard_output_frame = (
            self.get_parameter("hazard_output_frame").get_parameter_value().string_value
        )
        self.hazard_include_unknown = bool(
            self.get_parameter("hazard_include_unknown").value
        )
        self.hazard_stride = max(1, int(self.get_parameter("hazard_stride").value))
        self.hazard_output_topic_pc2 = (
            self.get_parameter("hazard_output_topic_pc2")
            .get_parameter_value()
            .string_value
        )

        # TF, subs/pubs
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        sensor_qos = QoSProfile(depth=5)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        sensor_qos.durability = DurabilityPolicy.VOLATILE
        self.pc2_sub = self.create_subscription(
            PointCloud2, self.cloud_topic, self.pc2_cb, sensor_qos
        )

        grid_qos = QoSProfile(depth=1)
        grid_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        grid_qos.reliability = ReliabilityPolicy.RELIABLE
        self.grid_pub = self.create_publisher(
            OccupancyGrid, "/traversability/occupancy", grid_qos
        )

        hazard_qos = QoSProfile(depth=10)
        hazard_qos.reliability = ReliabilityPolicy.RELIABLE
        hazard_qos.durability = DurabilityPolicy.VOLATILE
        self.hazard_pub = self.create_publisher(
            PointCloud, self.hazard_output_topic, hazard_qos
        )

        hazard_qos_pc2 = QoSProfile(depth=10)
        hazard_qos_pc2.reliability = ReliabilityPolicy.RELIABLE
        hazard_qos_pc2.durability = DurabilityPolicy.VOLATILE
        self.hazard_pub_pc2 = self.create_publisher(
            PointCloud2, self.hazard_output_topic_pc2, hazard_qos_pc2
        )

        self.get_logger().info(
            f"LocalTraversability started. Hazards -> {self.hazard_output_topic} ({self.hazard_output_frame})"
        )

    def _publish_hazard_cloud2(self, xyz, frame_id):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id
        msg = pc2.create_cloud_xyz32(header, xyz.tolist())
        self.hazard_pub_pc2.publish(msg)

    # -------------- input --------------
    def pc2_cb(self, msg: PointCloud2):
        src_frame = msg.header.frame_id
        try:
            pts = pc2.read_points_numpy(
                msg, field_names=("x", "y", "z"), skip_nans=True
            )
        except Exception as e:
            self.get_logger().error(f"Failed to read PointCloud2: {e}")
            return
        pts = np.asarray(pts, dtype=np.float32).reshape(-1, 3)
        if pts.size == 0:
            return
        self._run_pipeline(pts, src_frame)

    # -------------- pipeline (unchanged core logic) --------------
    def _run_pipeline(self, pts_src: np.ndarray, src_frame: str):
        try:
            tf = self.tf_buffer.lookup_transform(self.target_frame, src_frame, Time())
        except Exception as e:
            self.get_logger().warn(
                f"TF {self.target_frame} <- {src_frame} not ready: {e}"
            )
            return
        T = tf_to_matrix(tf)

        pts_h = np.hstack([pts_src, np.ones((pts_src.shape[0], 1), dtype=np.float32)])
        pts_t = (T @ pts_h.T).T[:, :3]

        if self.target_frame in ("base_link", "base_footprint"):
            try:
                tf_odom_bl = self.tf_buffer.lookup_transform(
                    "odom", "base_link", Time()
                )
                q = tf_odom_bl.transform.rotation
                roll, pitch, _ = quat2euler(q.x, q.y, q.z, q.w)
                R_unpitch_unroll = euler_to_matrix(-roll, -pitch, 0.0)
                pts_t = (R_unpitch_unroll @ pts_t.T).T
            except Exception as e:
                self.get_logger().warn(f"Gravity leveling skipped: {e}")

        x, y, z = pts_t[:, 0], pts_t[:, 1], pts_t[:, 2]

        m = (
            (x >= 0.0)
            & (x <= self.max_range)
            & (np.abs(y) <= self.half_width)
            & (z >= self.min_z)
            & (z <= self.max_z)
        )
        if not np.any(m):
            self._publish_hazard_cloud([], self.hazard_output_frame)
            return
        x, y, z = x[m], y[m], z[m]

        size = self.grid_size
        res = self.grid_res
        W = int(np.ceil(size / res))
        H = int(np.ceil(size / res))

        try:
            tf_bl = self.tf_buffer.lookup_transform(
                self.target_frame, "base_link", Time()
            )
            bl = tf_bl.transform.translation
            origin_x = float(bl.x) - size / 2.0
            origin_y = float(bl.y) - size / 2.0
        except Exception:
            origin_x = -size / 2.0
            origin_y = -size / 2.0

        col = np.floor((x - origin_x) / res).astype(np.int32)
        row = np.floor((y - origin_y) / res).astype(np.int32)
        ok = (col >= 0) & (col < W) & (row >= 0) & (row < H)
        if not np.any(ok):
            self._publish_hazard_cloud([], self.hazard_output_frame)
            return
        col, row, z = col[ok], row[ok], z[ok]

        Hmap = np.full((H, W), np.nan, dtype=np.float32)
        lin = row * W + col
        sum_z = np.zeros(H * W, dtype=np.float32)
        cnt_z = np.zeros(H * W, dtype=np.int32)
        np.add.at(sum_z, lin, z.astype(np.float32))
        np.add.at(cnt_z, lin, 1)
        valid = cnt_z > 0
        mean_z = np.full(H * W, np.nan, dtype=np.float32)
        mean_z[valid] = (sum_z[valid] / cnt_z[valid]).astype(np.float32)
        Hmap = mean_z.reshape(H, W)

        known = ~np.isnan(Hmap)
        unknown = np.isnan(Hmap)

        dz_dx = np.full_like(Hmap, np.nan, dtype=np.float32)
        dz_dy = np.full_like(Hmap, np.nan, dtype=np.float32)
        K = max(1, int(self.slope_baseline_cells))
        if W > 2 * K:
            mid_x = known[:, 2 * K :] & known[:, : -2 * K]
            dz_dx[:, K:-K][mid_x] = (
                Hmap[:, 2 * K :][mid_x] - Hmap[:, : -2 * K][mid_x]
            ) / (2.0 * K * res)
        if H > 2:
            mid_y = known[2:, :] & known[:-2, :]
            dz_dy[1:-1, :][mid_y] = (Hmap[2:, :][mid_y] - Hmap[:-2, :][mid_y]) / (
                2.0 * res
            )

        slope_mag = np.sqrt(np.square(dz_dx) + np.square(dz_dy))
        slope_mag_deg = np.degrees(np.arctan(slope_mag))
        slope_fwd_deg = np.degrees(np.arctan(dz_dx))

        H_for_max = np.nan_to_num(Hmap, nan=-np.inf)
        H_for_min = np.nan_to_num(Hmap, nan=+np.inf)
        Hmax = maximum_filter(H_for_max, size=3)
        Hmin = minimum_filter(H_for_min, size=3)
        Hmax[~np.isfinite(Hmax)] = np.nan
        Hmin[~np.isfinite(Hmin)] = np.nan

        step_relief = Hmax - Hmin
        step_haz = np.zeros_like(Hmap, dtype=bool)
        step_haz[known] = step_relief[known] > self.step_thresh_m

        holes = np.zeros_like(Hmap, dtype=bool)
        holes[known] = (Hmax[known] - Hmap[known]) > self.hole_thresh_m

        down_steps = np.zeros_like(Hmap, dtype=bool)
        Kmax = max(1, int(self.down_step_k_max))
        for k in range(1, Kmax + 1):
            if W <= k:
                continue
            pair_valid = known[:, :-k] & known[:, k:]
            diffk = np.full_like(Hmap, np.nan, dtype=np.float32)
            diffk[:, :-k][pair_valid] = (
                Hmap[:, :-k][pair_valid] - Hmap[:, k:][pair_valid]
            )
            down_steps[:, :-k] |= (
                np.nan_to_num(diffk[:, :-k], nan=-np.inf) > self.down_step_thresh_m
            )

        window_cells = max(3, int(round(self.gradual_drop_window_m / res)))
        gradual_down = np.zeros_like(known, dtype=bool)
        if W >= window_cells:
            dz = Hmap[:, 1:] - Hmap[:, :-1]
            valid_pairs = known[:, 1:] & known[:, :-1]
            dz[~valid_pairs] = 0.0
            drops = np.clip(-dz, 0.0, None)
            m = window_cells - 1
            if drops.shape[1] >= m:
                csum = np.cumsum(
                    np.pad(drops, ((0, 0), (1, 0)), constant_values=0.0), axis=1
                )
                moving = csum[:, m:] - csum[:, :-m]
                if moving.shape[1] > 0:
                    gradual_down[:, : moving.shape[1]] |= (
                        moving > self.gradual_drop_thresh_m
                    )

        ahead_cells = max(1, int(round(self.unknown_ahead_window_m / res)))
        count_unk = np.zeros_like(Hmap, dtype=np.int32)
        count_known_ahead = np.zeros_like(Hmap, dtype=np.int32)
        for k in range(1, ahead_cells + 1):
            if W > k:
                count_unk[:, :-k] += unknown[:, k:].astype(np.int32)
                count_known_ahead[:, :-k] += known[:, k:].astype(np.int32)
        unknown_frac = np.zeros_like(Hmap, dtype=np.float32)
        if ahead_cells > 0:
            unknown_frac[:, : W - ahead_cells] = count_unk[
                :, : W - ahead_cells
            ] / float(ahead_cells)
        unknown_ahead_dense = np.zeros_like(Hmap, dtype=bool)
        if W > ahead_cells:
            unknown_ahead_dense[:, : W - ahead_cells] = (
                (unknown_frac[:, : W - ahead_cells] >= self.unknown_ahead_frac_thresh)
                & (count_known_ahead[:, : W - ahead_cells] >= 1)
                & known[:, : W - ahead_cells]
            )

        haz_slope_down = np.isfinite(slope_fwd_deg) & (
            slope_fwd_deg < -self.downhill_deg_thresh
        )
        haz_slope_mag = np.isfinite(slope_mag_deg) & (
            slope_mag_deg > self.slope_deg_thresh
        )

        hazard = (
            holes
            | step_haz
            | down_steps
            | gradual_down
            | unknown_ahead_dense
            | haz_slope_down
            | haz_slope_mag
        )

        structure_side = max(3, int(round(0.10 / res)))
        structure = np.ones((structure_side, structure_side), dtype=bool)
        hazard = binary_opening(hazard, structure=structure)

        min_side_cells = max(1, int(round(self.min_block_window_m / res)))
        min_area_cells = min_side_cells * min_side_cells
        lbl, num = label(hazard, structure=structure)
        if num > 0:
            sizes = np.bincount(lbl.ravel())
            keep = sizes >= min_area_cells
            keep[0] = False
            hazard = keep[lbl]
        else:
            hazard = np.zeros_like(hazard, dtype=bool)

        inflate_cells = max(0, int(round(self.inflate_radius_m / res)))
        if inflate_cells > 0:
            se = np.ones((2 * inflate_cells + 1, 2 * inflate_cells + 1), dtype=bool)
            hazard = binary_dilation(hazard, structure=se)

        occ = np.full((H, W), -1, dtype=np.int8)
        free = (~hazard) & (~np.isnan(Hmap))
        occ[free] = 0
        occ[~free & (~np.isnan(Hmap))] = 100
        occ[hazard] = 100

        grid = OccupancyGrid()
        grid.header = Header()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = self.target_frame
        grid.info.resolution = res
        grid.info.width = W
        grid.info.height = H
        grid.info.origin.position.x = origin_x
        grid.info.origin.position.y = origin_y
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation.w = 1.0
        grid.data = occ.flatten(order="C").tolist()
        self.grid_pub.publish(grid)

        haz_mask = np.array(hazard, dtype=bool)
        if self.hazard_include_unknown:
            haz_mask |= occ < 0

        if self.hazard_stride > 1:
            tmp = np.zeros_like(haz_mask, dtype=bool)
            tmp[0 :: self.hazard_stride, 0 :: self.hazard_stride] = True
            haz_mask &= tmp

        rr, cc = np.where(haz_mask)
        if rr.size == 0:
            self._publish_hazard_cloud([], self.hazard_output_frame)
            return

        xg = origin_x + (cc.astype(np.float32) + 0.5) * res
        yg = origin_y + (rr.astype(np.float32) + 0.5) * res
        zg = np.zeros_like(xg, dtype=np.float32)

        try:
            tf_out = self.tf_buffer.lookup_transform(
                self.hazard_output_frame, self.target_frame, Time()
            )
        except Exception as e:
            self.get_logger().warn(
                f"TF {self.hazard_output_frame} <- {self.target_frame} not ready: {e}"
            )
            return

        T_out = tf_to_matrix(tf_out)
        pts_grid = np.vstack([xg, yg, zg, np.ones_like(xg)]).astype(np.float32)
        pts_out = (T_out @ pts_grid).T[:, :3]

        points = [Point32(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in pts_out]
        self._publish_hazard_cloud(points, self.hazard_output_frame)
        self._publish_hazard_cloud2(
            pts_out.astype(np.float32), self.hazard_output_frame
        )

    # -------------- pubs --------------
    def _publish_hazard_cloud(self, points, frame_id):
        pc = PointCloud()
        pc.header.frame_id = frame_id
        pc.header.stamp = self.get_clock().now().to_msg()
        pc.points = points
        self.hazard_pub.publish(pc)


# ---------------- main ----------------


def main():
    # Start helpers before initializing ROS to surface launch errors early
    # (this also lets their logs flow while we spin)
    tmp_logger = None
    try:
        rclpy.init()
        # create a temporary node just to log helper starts nicely
        tmp_node = rclpy.create_node("local_traversability_launcher")
        tmp_logger = tmp_node.get_logger()
        _launch_ros2_helpers(tmp_logger)
        tmp_node.destroy_node()
    except Exception:
        # If helpers failed to start, ensure ROS is shut down and re-raise
        try:
            rclpy.shutdown()
        except Exception:
            pass
        raise

    node = LocalTraversability()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down…")
        node.destroy_node()
        rclpy.shutdown()
        _kill_process_tree()


if __name__ == "__main__":
    main()
