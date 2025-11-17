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
    """Create a straight line path from origin at specified angle and length"""
    angle_rad = math.radians(angle_degrees)
    end_x = length * math.cos(angle_rad)  # +X forward
    end_y = length * math.sin(angle_rad)  # +Y left

    x_vals = np.linspace(0.0, end_x, num_points)
    y_vals = np.linspace(0.0, end_y, num_points)
    return np.array([x_vals, y_vals])


# Define 9 straight line paths separated by 15 degrees
# Center path is 0° (straight forward), then ±15°, ±30°, ±45°, ±60°
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

        self.half_width_robot = 0.20
        self.sensor_mounting_angle = 172.0
        self.relevant_distance_min = 0.20
        self.obstacle_threshold = 0.50  # 50 data points
        self.laser_frame = "laser"

        self.scan = None
        self.obstacle: PointCloud | None = None
        self.hazard_xy_robot = np.zeros((0, 2), dtype=np.float32)

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
            PointCloud2, "/traversability/hazard_points2", self.hazard_callback_pc2, 10
        )

        self.paths_pub = self.create_publisher(Paths, "/om/paths", 10)
        self.markers_pub = self.create_publisher(MarkerArray, "/om/paths_markers", 10)

        self.get_logger().info("OMPath node started")

    def scan_callback(self, msg: LaserScan):
        self.scan = msg

        angles = list(
            map(
                lambda x: 360.0 * (x + math.pi) / (2 * math.pi) - 180.0,
                np.arange(
                    self.scan.angle_min,
                    self.scan.angle_max,
                    self.scan.angle_increment,
                ),
            )
        )
        angles_final = angles
        # angles now run from 360.0 to 0 degress
        data = list(zip(angles_final, self.scan.ranges))

        complexes = []

        for angle, distance in data:
            d_m = distance

            if not math.isfinite(d_m) or d_m > 5.0 or d_m < self.relevant_distance_min:
                continue

            # first, correctly orient the sensor zero to the robot zero
            angle = angle + self.sensor_mounting_angle
            if angle >= 360.0:
                angle = angle - 360.0
            elif angle < 0.0:
                angle = 360.0 + angle

            # then, convert to radians
            a_rad = angle * math.pi / 180.0

            x = d_m * math.cos(a_rad)
            y = d_m * math.sin(a_rad)

            angle = angle - 180.0

            # the final data ready to use for path planning
            complexes.append([x, y, angle, d_m])

        if self.obstacle and len(self.obstacle.points) > self.obstacle_threshold:
            for p in self.obstacle.points:
                x = float(p.x)
                y = float(p.y)

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
        possible_paths = np.array([0, 1, 2, 3, 4, 5, 6, 7, 8, 9])
        bad_paths = []
        blocked_by_obstacle_set = set()
        blocked_by_hazard_set = set()

        for x, y, d in list(zip(X, Y, D)):
            for apath in possible_paths.copy():
                # For going back, only consider obstacles that are:
                # 1. Behind the robot (negative x in robot frame)
                # 2. Within a certain distance threshold

                # Assuming robot faces positive x direction
                # Only check obstacles behind the robot
                if (
                    apath == 9 and x >= 0.0
                ):  # Skip obstacles in front of or beside the robot
                    continue
                # Get the start and end points of this straight line path
                path_points = paths[apath]
                start_x, start_y = path_points[0][0], path_points[1][0]
                end_x, end_y = path_points[0][-1], path_points[1][-1]
                # Calculate distance from obstacle to the line segment
                dist_to_line = self.distance_point_to_line_segment(
                    x, y, start_x, start_y, end_x, end_y
                )
                if dist_to_line < self.half_width_robot:
                    # too close - this path will not work
                    path_to_remove = np.array([apath])
                    bad_paths.append(apath)
                    blocked_by_obstacle_set.add(int(apath))
                    # Remove this path from the possible paths
                    # np.setdiff1d returns a new array, so we need to update possible_paths
                    possible_paths = np.setdiff1d(possible_paths, path_to_remove)
                    break

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
                        path_to_remove = np.array([apath])
                        bad_paths.append(apath)
                        blocked_by_hazard_set.add(int(apath))
                        possible_paths = np.setdiff1d(possible_paths, path_to_remove)
                        break

        blocked_by_obstacle_idx = [int(i) for i in sorted(blocked_by_obstacle_set)]
        blocked_by_hazard_idx = [int(i) for i in sorted(blocked_by_hazard_set)]

        paths_msg = Paths()
        paths_msg.header.stamp = self.get_clock().now().to_msg()
        paths_msg.header.frame_id = self.laser_frame
        paths_msg.paths = possible_paths.tolist()

        paths_msg.blocked_by_obstacle_idx = blocked_by_obstacle_idx
        paths_msg.blocked_by_hazard_idx = blocked_by_hazard_idx

        self.paths_pub.publish(paths_msg)

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
        # Vector from line start to line end
        dx = x2 - x1
        dy = y2 - y1

        # If the line segment has zero length, return distance to point
        if dx == 0 and dy == 0:
            return math.sqrt((px - x1) ** 2 + (py - y1) ** 2)

        # Calculate the parameter t that represents the projection of the point onto the line
        t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)

        # Clamp t to [0, 1] to stay within the line segment
        t = max(0, min(1, t))

        # Find the closest point on the line segment
        closest_x = x1 + t * dx
        closest_y = y1 + t * dy

        # Return the distance from the point to the closest point on the line segment
        return math.sqrt((px - closest_x) ** 2 + (py - closest_y) ** 2)

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
