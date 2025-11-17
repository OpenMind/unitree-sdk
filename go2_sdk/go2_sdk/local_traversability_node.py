import math

import geometry_msgs.msg
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
    label,
    maximum_filter,
    minimum_filter,
)
from sensor_msgs.msg import PointCloud, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header


def tf_to_matrix(tf: geometry_msgs.msg.TransformStamped) -> np.ndarray:
    """Convert a ROS2 TransformStamped into a 4x4 homogeneous transform matrix.

    Parameters
    ----------
    tf : geometry_msgs.msg.TransformStamped
        Transform from source frame to target frame.

    Returns
    -------
    np.ndarray
        (4, 4) homogeneous transform matrix mapping points from source to target.
    """
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


def quat2euler(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    """Convert quaternion to roll, pitch, yaw (in radians).

    Parameters
    ----------
    x, y, z, w : float
        Quaternion components.

    Returns
    -------
    tuple[float, float, float]
        (roll, pitch, yaw) in radians.
    """
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + z * z)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1 else math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def euler_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Build a 3x3 rotation matrix from roll, pitch, yaw.

    Parameters
    ----------
    roll, pitch, yaw : float
        Euler angles in radians.

    Returns
    -------
    np.ndarray
        (3, 3) rotation matrix.
    """
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=np.float32)
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=np.float32)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=np.float32)
    return (Rz @ Ry @ Rx).astype(np.float32)


class LocalTraversability(Node):
    """Build a local traversability grid directly from depth point clouds.

    High-level behavior
    -------------------
    1. Transform depth point cloud into a world frame (e.g. `odom`).
    2. Around the robot, build a 5 m x 5 m height map (grid of mean z).
    3. Classify hazardous cells as:
       - steep slopes (high gradient),
       - local steps / holes (big height jump in 3x3),
    4. Publish:
       - nav_msgs/OccupancyGrid for visualization / debugging,
       - hazard point clouds (PointCloud + PointCloud2) for the path selector.
    """

    def __init__(self):
        """Initialize the LocalTraversability node and its ROS interfaces."""
        super().__init__("local_traversability")

        # Input depth cloud topic and frames
        self.depth_cloud_topic = "/camera/depth/points"
        # Global/local frame in which we build the grid (e.g. 'odom' or 'map')
        self.map_frame = "odom"

        # Local grid geometry & region of interest in robot-centric frame
        self.local_grid_resolution_m = 0.05  # cell size (5 cm)
        self.local_grid_size_m = 5.0  # 5 m x 5 m grid
        self.max_forward_range_m = 2.0  # only consider points within 2 m in front
        self.half_lateral_fov_m = 2.5  # +/- 2.5 m sideways (covers 5 m width)
        self.min_height_m = -1.0  # ignore points below this height
        self.max_height_m = 2.0  # ignore points above this height

        # Slope thresholds (for continuous sloped surfaces)
        self.max_allowed_slope_deg = (
            10.0  # any local slope > 10° is considered hazardous
        )
        self.max_forward_downhill_deg = (
            8.0  # >= 8° downhill in forward direction is hazardous
        )
        self.slope_baseline_cells = 3  # use 3 cells (~15 cm) as baseline spacing

        # Local height discontinuity (steps / stairs / holes) in a 3x3 neighborhood
        self.local_relief_step_thresh_m = 0.08  # > 8 cm height difference => hazard

        # Minimum size of a connected hazardous blob (remove tiny speckles)
        # Any connected hazard region smaller than [min_hazard_blob_side_m]^2 is removed.
        self.min_hazard_blob_side_m = 0.20  # 30 cm x 30 cm blob minimum

        # Hazard inflation radius: extra safety margin around hazards
        self.hazard_inflation_radius_m = 0.0
        # Hazard outputs

        self.hazard_points_frame = (
            "laser"  # hazard points expressed in this frame (for om_path)
        )
        self.hazard_thinning_stride = 1  # 1 => keep every cell, >1 => sub-sample
        self.hazard_points_topic_pc2 = "/traversability/hazard_points2"

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Depth cloud subscriber
        sensor_qos = QoSProfile(depth=5)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        sensor_qos.durability = DurabilityPolicy.VOLATILE
        self.pc2_sub = self.create_subscription(
            PointCloud2, self.depth_cloud_topic, self.pc2_call_back, sensor_qos
        )

        # Occupancy grid publisher (latched / transient local)
        grid_qos = QoSProfile(depth=1)
        grid_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        grid_qos.reliability = ReliabilityPolicy.RELIABLE
        self.grid_pub = self.create_publisher(
            OccupancyGrid, "/traversability/occupancy", grid_qos
        )

        # Hazard point cloud publishers
        hazard_qos = QoSProfile(depth=10)
        hazard_qos.reliability = ReliabilityPolicy.RELIABLE
        hazard_qos.durability = DurabilityPolicy.VOLATILE

        hazard_qos_pc2 = QoSProfile(depth=10)
        hazard_qos_pc2.reliability = ReliabilityPolicy.RELIABLE
        hazard_qos_pc2.durability = DurabilityPolicy.VOLATILE
        self.hazard_pub_pc2 = self.create_publisher(
            PointCloud2, self.hazard_points_topic_pc2, hazard_qos_pc2
        )

        self.get_logger().info(
            f"LocalTraversability started. Hazards -> {self.hazard_points_topic_pc2} "
            f"({self.hazard_points_frame})"
        )

    def _publish_hazard_cloud2(self, xyz: np.ndarray, frame_id: str) -> None:
        """Publish a PointCloud2 of hazard cell centers.

        Parameters
        ----------
        xyz : np.ndarray
            Array of shape (N, 3) with hazard point coordinates in `frame_id`.
        frame_id : str
            TF frame in which the points are expressed.
        """
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id
        msg = pc2.create_cloud_xyz32(header, xyz.tolist())
        self.hazard_pub_pc2.publish(msg)

    def pc2_call_back(self, msg: PointCloud2) -> None:
        """Callback for incoming depth PointCloud2.

        Reads (x, y, z) points, converts them to numpy, and runs the
        traversability pipeline.

        Parameters
        ----------
        msg : sensor_msgs.msg.PointCloud2
            Incoming depth point cloud in camera frame.
        """
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

    def _run_pipeline(self, pts_src: np.ndarray, src_frame: str) -> None:
        """Full traversability computation pipeline.

        Steps
        -----
        1. Transform point cloud into `map_frame`.
        2. Get base_link pose in `map_frame`.
        3. Crop points to a rectangular ROI in front of the robot.
        4. Build a local height map (mean z per cell).
        5. Compute:
           - slope-based hazards (large sloped regions),
           - local step/hole hazards (3x3 height relief),
        6. Merge hazards, filter small blobs, optionally inflate.
        7. Publish OccupancyGrid and hazard point clouds.

        Parameters
        ----------
        pts_src : np.ndarray
            Input points in src_frame, shape (N, 3).
        src_frame : str
            Name of the TF frame for the input cloud.
        """

        try:
            tf_cloud = self.tf_buffer.lookup_transform(
                self.map_frame, src_frame, Time()
            )
        except Exception as e:
            self.get_logger().warn(f"TF {self.map_frame} <- {src_frame} not ready: {e}")
            return
        T_cloud = tf_to_matrix(tf_cloud)

        pts_h = np.hstack([pts_src, np.ones((pts_src.shape[0], 1), dtype=np.float32)])
        pts_t = (T_cloud @ pts_h.T).T[:, :3]  # points in map_frame

        try:
            tf_bl = self.tf_buffer.lookup_transform(self.map_frame, "base_link", Time())
            bl_t = tf_bl.transform.translation
            bl_q = tf_bl.transform.rotation
            bl_x = float(bl_t.x)
            bl_y = float(bl_t.y)
            _, _, yaw = quat2euler(bl_q.x, bl_q.y, bl_q.z, bl_q.w)
        except Exception as e:
            self.get_logger().warn(f"TF {self.map_frame} <- base_link not ready: {e}")
            bl_x = 0.0
            bl_y = 0.0
            yaw = 0.0

        x_world, y_world, z_world = pts_t[:, 0], pts_t[:, 1], pts_t[:, 2]

        # World -> robot frame: translate so robot is at origin, then rotate by -yaw
        dx = x_world - bl_x
        dy = y_world - bl_y
        cos_neg_yaw = math.cos(-yaw)
        sin_neg_yaw = math.sin(-yaw)
        x_robot = cos_neg_yaw * dx - sin_neg_yaw * dy  # forward
        y_robot = sin_neg_yaw * dx + cos_neg_yaw * dy  # left

        roi_mask = (
            (x_robot >= 0.0)
            & (x_robot <= self.max_forward_range_m)
            & (np.abs(y_robot) <= self.half_lateral_fov_m)
            & (z_world >= self.min_height_m)
            & (z_world <= self.max_height_m)
        )
        if not np.any(roi_mask):
            # Publish empty hazard cloud to clear stale data downstream
            self._publish_hazard_cloud2(
                np.empty((0, 3), dtype=np.float32),
                self.hazard_points_frame,
            )
            return

        # ROI-filtered world coordinates (still in map_frame)
        x_world = x_world[roi_mask]
        y_world = y_world[roi_mask]
        z_world = z_world[roi_mask]

        # Local grid geometry: 5 m x 5 m centered on base_link
        grid_size_m = self.local_grid_size_m
        res = self.local_grid_resolution_m
        grid_width = int(math.ceil(grid_size_m / res))
        grid_height = int(math.ceil(grid_size_m / res))

        # Origin (bottom-left corner) in map_frame; robot is at the grid center
        origin_x = bl_x - grid_size_m / 2.0
        origin_y = bl_y - grid_size_m / 2.0

        # map_frame -> grid indices
        col = np.floor((x_world - origin_x) / res).astype(np.int32)
        row = np.floor((y_world - origin_y) / res).astype(np.int32)
        in_bounds = (col >= 0) & (col < grid_width) & (row >= 0) & (row < grid_height)
        if not np.any(in_bounds):
            self._publish_hazard_cloud2(
                np.empty((0, 3), dtype=np.float32),
                self.hazard_points_frame,
            )
            return
        col, row, z_world = col[in_bounds], row[in_bounds], z_world[in_bounds]

        # Build height map: mean z per cell
        height_map = np.full((grid_height, grid_width), np.nan, dtype=np.float32)
        lin_idx = row * grid_width + col
        sum_z = np.zeros(grid_height * grid_width, dtype=np.float32)
        cnt_z = np.zeros(grid_height * grid_width, dtype=np.int32)
        np.add.at(sum_z, lin_idx, z_world.astype(np.float32))
        np.add.at(cnt_z, lin_idx, 1)
        valid = cnt_z > 0
        mean_z = np.full(grid_height * grid_width, np.nan, dtype=np.float32)
        mean_z[valid] = (sum_z[valid] / cnt_z[valid]).astype(np.float32)
        height_map = mean_z.reshape(grid_height, grid_width)

        known_mask = ~np.isnan(height_map)

        # Gradients / slope on the height map
        dz_dx = np.full_like(height_map, np.nan, dtype=np.float32)
        dz_dy = np.full_like(height_map, np.nan, dtype=np.float32)
        K = max(1, int(self.slope_baseline_cells))

        # Forward-backward slope (x direction)
        if grid_width > 2 * K:
            mid_x = known_mask[:, 2 * K :] & known_mask[:, : -2 * K]
            dz_dx[:, K:-K][mid_x] = (
                height_map[:, 2 * K :][mid_x] - height_map[:, : -2 * K][mid_x]
            ) / (2.0 * K * res)

        # Left-right slope (y direction)
        if grid_height > 2:
            mid_y = known_mask[2:, :] & known_mask[:-2, :]
            dz_dy[1:-1, :][mid_y] = (
                height_map[2:, :][mid_y] - height_map[:-2, :][mid_y]
            ) / (2.0 * res)

        slope_mag = np.sqrt(np.square(dz_dx) + np.square(dz_dy))
        slope_mag_deg = np.degrees(np.arctan(slope_mag))
        slope_fwd_deg = np.degrees(np.arctan(dz_dx))

        # Slope-based hazards: global steep regions
        haz_slope_down = np.isfinite(slope_fwd_deg) & (
            slope_fwd_deg < -self.max_forward_downhill_deg
        )
        haz_slope_mag = np.isfinite(slope_mag_deg) & (
            slope_mag_deg > self.max_allowed_slope_deg
        )

        # Local height relief: 3x3 window; big delta -> step/hole hazard
        h_for_max = np.nan_to_num(height_map, nan=-np.inf)
        h_for_min = np.nan_to_num(height_map, nan=+np.inf)
        local_max = maximum_filter(h_for_max, size=3)
        local_min = minimum_filter(h_for_min, size=3)
        local_max[~np.isfinite(local_max)] = np.nan
        local_min[~np.isfinite(local_min)] = np.nan

        local_relief = local_max - local_min
        step_hazard = np.zeros_like(height_map, dtype=bool)
        step_hazard[known_mask] = (
            local_relief[known_mask] > self.local_relief_step_thresh_m
        )

        # Combine hazards from all sources
        hazard = step_hazard | haz_slope_down | haz_slope_mag

        # De-speckle: keep only blobs with area >= (min_hazard_blob_side_m)^2
        structure = np.ones((3, 3), dtype=bool)  # 8-connected components
        lbl, num = label(hazard, structure=structure)
        if num > 0:
            sizes = np.bincount(lbl.ravel())
            min_side_cells = max(1, int(round(self.min_hazard_blob_side_m / res)))
            min_area_cells = min_side_cells * min_side_cells
            keep = sizes >= min_area_cells
            keep[0] = False  # background label index
            hazard = keep[lbl]
        else:
            hazard = np.zeros_like(hazard, dtype=bool)

        # Optional dilation around hazard blobs (currently zero radius => no-op)
        inflate_cells = max(0, int(round(self.hazard_inflation_radius_m / res)))
        if inflate_cells > 0:
            se = np.ones((2 * inflate_cells + 1, 2 * inflate_cells + 1), dtype=bool)
            hazard = binary_dilation(hazard, structure=se)

        # Build OccupancyGrid (-1 unknown, 0 free, 100 hazard/occupied)
        occ = np.full((grid_height, grid_width), -1, dtype=np.int8)
        free_mask = (~hazard) & (~np.isnan(height_map))
        occ[free_mask] = 0
        occ[~free_mask & (~np.isnan(height_map))] = 100
        occ[hazard] = 100

        grid = OccupancyGrid()
        grid.header = Header()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = self.map_frame
        grid.info.resolution = res
        grid.info.width = grid_width
        grid.info.height = grid_height
        grid.info.origin.position.x = origin_x
        grid.info.origin.position.y = origin_y
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation.w = 1.0
        grid.data = occ.flatten(order="C").tolist()
        self.grid_pub.publish(grid)

        # Hazard point clouds (for om_path)
        haz_mask = np.array(hazard, dtype=bool)

        # Thin points by stride if desired
        if self.hazard_thinning_stride > 1:
            tmp = np.zeros_like(haz_mask, dtype=bool)
            tmp[0 :: self.hazard_thinning_stride, 0 :: self.hazard_thinning_stride] = (
                True
            )
            haz_mask &= tmp

        rr, cc = np.where(haz_mask)
        if rr.size == 0:
            self._publish_hazard_cloud2(
                np.empty((0, 3), dtype=np.float32),
                self.hazard_points_frame,
            )
            return

        xg = origin_x + (cc.astype(np.float32) + 0.5) * res
        yg = origin_y + (rr.astype(np.float32) + 0.5) * res
        zg = np.zeros_like(xg, dtype=np.float32)

        # Transform hazard cell centers from map_frame -> hazard_points_frame (e.g. 'laser')
        try:
            tf_out = self.tf_buffer.lookup_transform(
                self.hazard_points_frame, self.map_frame, Time()
            )
        except Exception as e:
            self.get_logger().warn(
                f"TF {self.hazard_points_frame} <- {self.map_frame} not ready: {e}"
            )
            return

        T_out = tf_to_matrix(tf_out)
        pts_grid = np.vstack([xg, yg, zg, np.ones_like(xg)]).astype(np.float32)
        pts_out = (T_out @ pts_grid).T[:, :3]

        self._publish_hazard_cloud2(
            pts_out.astype(np.float32), self.hazard_points_frame
        )


def main():
    """Entry point for the local_traversability node."""
    rclpy.init()
    node = LocalTraversability()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
