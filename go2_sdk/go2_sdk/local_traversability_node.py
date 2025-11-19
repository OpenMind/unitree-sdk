import math

import geometry_msgs.msg
import numpy as np
import rclpy
import tf2_ros
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
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header


def tf_to_matrix(tf: geometry_msgs.msg.TransformStamped) -> np.ndarray:
    """
    Convert a TransformStamped into a 4x4 homogeneous transform matrix.

    Parameters
    ----------
    tf : geometry_msgs.msg.TransformStamped
        Transform from source frame to target frame.

    Returns
    -------
    np.ndarray
        4x4 matrix mapping points from source to target.
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


class LocalTraversability(Node):
    """
    Build a local traversability grid directly in `base_link` frame.

    High-level pipeline
    -------------------
    1. Transform incoming depth cloud from camera frame into `base_link`.
    2. Around `base_link`, build a 5 m x 5 m local height map (mean z per cell).
    3. Detect hazardous cells from:
       - surface slope (gradients),
       - local height jumps / steps / holes.
    4. Publish:
       - nav_msgs/OccupancyGrid (frame_id = base_link),
       - hazard_points2 (sensor_msgs/PointCloud2, frame_id = base_link).
    """

    def __init__(self):
        super().__init__("local_traversability")

        # Input depth cloud topic and frames
        self.depth_cloud_topic = "/camera/depth/points"

        # We work directly in the robot frame
        self.map_frame = "base_link"
        self.hazard_points_frame = "base_link"

        # Local grid geometry & region of interest in robot-centric frame
        self.local_grid_resolution_m = 0.05  # cell size (5 cm)
        self.local_grid_size_m = 5.0  # 5 m x 5 m grid
        self.max_forward_range_m = 2.0  # only consider points within 2 m in front
        self.half_lateral_fov_m = 2.5  # +/- 2.5 m sideways (covers 5 m width)
        self.min_height_m = -1.0  # ignore points below this height
        self.max_height_m = 2.0  # ignore points above this height

        # Slope thresholds (for continuous sloped surfaces)
        self.max_allowed_slope_deg = 10.0  # any slope > 10° is hazardous
        self.max_forward_downhill_deg = (
            8.0  # >= 8° downhill in forward direction is hazardous
        )
        self.slope_baseline_cells = 3  # use 3 cells (~15 cm) as baseline spacing

        # Local height discontinuity (steps / stairs / holes) in a 3x3 neighborhood
        self.local_relief_step_thresh_m = 0.12  # > 8 cm height difference => hazard

        # Minimum size of a connected hazardous blob (remove tiny speckles)
        # Any connected hazard region smaller than [min_hazard_blob_side_m]^2 is removed.
        self.min_hazard_blob_side_m = 0.30  # 30 cm x 30 cm blob minimum

        # Hazard inflation radius: extra safety margin around hazards
        self.hazard_inflation_radius_m = 0.0

        # Hazard output configuration
        self.hazard_thinning_stride = 1  # 1 => keep every cell, >1 => sub-sample
        self.hazard_points_topic_pc2 = "/traversability/hazard_points2"

        # TF buffer / listener
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

        # Hazard PointCloud2 publisher
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
        """
        Publish a PointCloud2 of hazard cell centers.

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
        """
        Callback for incoming depth PointCloud2.

        Reads (x, y, z) points, converts them to numpy, and runs the
        traversability pipeline.

        Parameters
        ----------
        msg : sensor_msgs.msg.PointCloud2
            Incoming depth point cloud in camera frame.
        """
        source_frame = msg.header.frame_id  # camera frame
        try:
            points = pc2.read_points_numpy(
                msg, field_names=("x", "y", "z"), skip_nans=True
            )
        except Exception as e:
            self.get_logger().error(f"Failed to read PointCloud2: {e}")
            return

        points = np.asarray(points, dtype=np.float32).reshape(-1, 3)
        if points.size == 0:
            return

        self._run_pipeline(points, source_frame)

    def _run_pipeline(
        self,
        points_in_source_frame: np.ndarray,
        source_frame: str,
    ) -> None:
        """
        Run the full traversability computation:

        1. Transform depth cloud from camera frame to `base_link`.
        2. Filter points to a region-of-interest (ROI) around the robot.
        3. Build a local height map around base_link.
        4. Compute slope-based and step-based hazards.
        5. Publish OccupancyGrid and hazard_points2 (all in base_link).

        Parameters
        ----------
        points_in_source_frame : np.ndarray
            Input points in src_frame, shape (N, 3).
        source_frame : str
            Name of the TF frame for the input cloud.
        """
        # Transform camera -> base_link (self.map_frame)
        try:
            transform_cloud_to_robot = self.tf_buffer.lookup_transform(
                self.map_frame,
                source_frame,
                Time(),
            )
        except Exception as e:
            self.get_logger().warn(
                f"TF {self.map_frame} <- {source_frame} not ready: {e}"
            )
            return

        T_cloud = tf_to_matrix(transform_cloud_to_robot)

        homogeneous_points = np.hstack(
            [
                points_in_source_frame,
                np.ones((points_in_source_frame.shape[0], 1), dtype=np.float32),
            ]
        )
        # Points in robot frame (base_link)
        points_robot = (T_cloud @ homogeneous_points.T).T[:, :3]  # (N, 3)

        x_robot = points_robot[:, 0]
        y_robot = points_robot[:, 1]
        z_robot = points_robot[:, 2]

        # ROI filter in robot frame: only points near the robot
        roi_mask = (
            (x_robot >= 0.0)
            & (x_robot <= self.max_forward_range_m)
            & (np.abs(y_robot) <= self.half_lateral_fov_m)
            & (z_robot >= self.min_height_m)
            & (z_robot <= self.max_height_m)
        )
        if not np.any(roi_mask):
            # Publish an empty hazard cloud to clear stale data downstream
            self._publish_hazard_cloud2(
                np.empty((0, 3), dtype=np.float32),
                self.hazard_points_frame,
            )
            return

        x_roi = x_robot[roi_mask]
        y_roi = y_robot[roi_mask]
        z_roi = z_robot[roi_mask]

        # Local grid geometry: 5 m x 5 m centered on base_link
        grid_size_m = self.local_grid_size_m
        res = self.local_grid_resolution_m
        grid_width = int(math.ceil(grid_size_m / res))
        grid_height = int(math.ceil(grid_size_m / res))

        # Origin (bottom-left corner) in map_frame; robot is at the grid center
        origin_x = -grid_size_m / 2.0
        origin_y = -grid_size_m / 2.0

        col = np.floor((x_roi - origin_x) / res).astype(np.int32)
        row = np.floor((y_roi - origin_y) / res).astype(np.int32)
        in_bounds = (col >= 0) & (col < grid_width) & (row >= 0) & (row < grid_height)
        if not np.any(in_bounds):
            self._publish_hazard_cloud2(
                np.empty((0, 3), dtype=np.float32),
                self.hazard_points_frame,
            )
            return

        col = col[in_bounds]
        row = row[in_bounds]
        z_roi = z_roi[in_bounds]

        # Height map: mean z per cell
        height_map = np.full((grid_height, grid_width), np.nan, dtype=np.float32)
        linear_indices = row * grid_width + col
        sum_z = np.zeros(grid_height * grid_width, dtype=np.float32)
        count_z = np.zeros(grid_height * grid_width, dtype=np.int32)

        np.add.at(sum_z, linear_indices, z_roi.astype(np.float32))
        np.add.at(count_z, linear_indices, 1)

        valid_cells = count_z > 0
        mean_z = np.full(grid_height * grid_width, np.nan, dtype=np.float32)
        mean_z[valid_cells] = (sum_z[valid_cells] / count_z[valid_cells]).astype(
            np.float32
        )
        height_map = mean_z.reshape(grid_height, grid_width)

        known_mask = ~np.isnan(height_map)

        # Gradients / slope on the height map
        dz_dx = np.full_like(height_map, np.nan, dtype=np.float32)
        dz_dy = np.full_like(height_map, np.nan, dtype=np.float32)
        baseline_cells = max(1, int(self.slope_baseline_cells))

        # orward-backward slope (x direction)
        if grid_width > 2 * baseline_cells:
            mid_x_mask = (
                known_mask[:, 2 * baseline_cells :]
                & known_mask[:, : -2 * baseline_cells]
            )
            dz_dx[:, baseline_cells:-baseline_cells][mid_x_mask] = (
                height_map[:, 2 * baseline_cells :][mid_x_mask]
                - height_map[:, : -2 * baseline_cells][mid_x_mask]
            ) / (2.0 * baseline_cells * res)

        # Left-right slope (y direction)
        if grid_height > 2:
            mid_y_mask = known_mask[2:, :] & known_mask[:-2, :]
            dz_dy[1:-1, :][mid_y_mask] = (
                height_map[2:, :][mid_y_mask] - height_map[:-2, :][mid_y_mask]
            ) / (2.0 * res)

        slope_mag = np.sqrt(np.square(dz_dx) + np.square(dz_dy))
        slope_mag_deg = np.degrees(np.arctan(slope_mag))
        slope_fwd_deg = np.degrees(np.arctan(dz_dx))

        # Slope-based hazards
        hazard_slope_down = np.isfinite(slope_fwd_deg) & (
            slope_fwd_deg < -self.max_forward_downhill_deg
        )
        hazard_slope_mag = np.isfinite(slope_mag_deg) & (
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
        hazard = step_hazard | hazard_slope_down | hazard_slope_mag

        # De-speckle: keep only blobs with area >= (min_hazard_blob_side_m)^2
        structure = np.ones((3, 3), dtype=bool)
        labels, num_labels = label(hazard, structure=structure)
        if num_labels > 0:
            sizes = np.bincount(labels.ravel())
            min_side_cells = max(1, int(round(self.min_hazard_blob_side_m / res)))
            min_area_cells = min_side_cells * min_side_cells
            keep = sizes >= min_area_cells
            keep[0] = False  # background
            hazard = keep[labels]
        else:
            hazard = np.zeros_like(hazard, dtype=bool)

        # Optional dilation around hazard blobs (currently zero radius => no-op)
        inflate_cells = max(0, int(round(self.hazard_inflation_radius_m / res)))
        if inflate_cells > 0:
            se = np.ones((2 * inflate_cells + 1, 2 * inflate_cells + 1), dtype=bool)
            hazard = binary_dilation(hazard, structure=se)

        # Build OccupancyGrid (-1 unknown, 0 free, 100 hazard/occupied)
        occupancy = np.full((grid_height, grid_width), -1, dtype=np.int8)
        free_mask = (~hazard) & (~np.isnan(height_map))
        occupancy[free_mask] = 0
        occupancy[~free_mask & (~np.isnan(height_map))] = 100
        occupancy[hazard] = 100

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
        grid.data = occupancy.flatten(order="C").tolist()
        self.grid_pub.publish(grid)

        # Hazard point clouds (for om_path)
        hazard_mask = np.array(hazard, dtype=bool)

        # Thin points by stride if desired
        if self.hazard_thinning_stride > 1:
            thinning_mask = np.zeros_like(hazard_mask, dtype=bool)
            thinning_mask[
                0 :: self.hazard_thinning_stride,
                0 :: self.hazard_thinning_stride,
            ] = True
            hazard_mask &= thinning_mask

        rr, cc = np.where(hazard_mask)
        if rr.size == 0:
            self._publish_hazard_cloud2(
                np.empty((0, 3), dtype=np.float32),
                self.hazard_points_frame,
            )
            return

        xg = origin_x + (cc.astype(np.float32) + 0.5) * res
        yg = origin_y + (rr.astype(np.float32) + 0.5) * res
        zg = np.zeros_like(xg, dtype=np.float32)

        hazard_points_robot = np.stack([xg, yg, zg], axis=1)
        self._publish_hazard_cloud2(
            hazard_points_robot.astype(np.float32),
            self.hazard_points_frame,
        )


def main():
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
