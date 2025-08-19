import math

import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud
from om_api.msg import Paths

def create_straight_line_path_from_angle(angle_degrees, length=1.0, num_points=10):
    """Create a straight line path from origin at specified angle and length"""
    angle_rad = math.radians(angle_degrees)
    end_x = length * math.sin(angle_rad)  # sin for x because 0° is forward (positive y)
    end_y = length * math.cos(angle_rad)  # cos for y because 0° is forward (positive y)

    x_vals = np.linspace(0.0, end_x, num_points)
    y_vals = np.linspace(0.0, end_y, num_points)
    return np.array([x_vals, y_vals])


# Define 9 straight line paths separated by 15 degrees
# Center path is 0° (straight forward), then ±15°, ±30°, ±45°, ±60°
path_angles = [-60, -45, -30, -15, 0, 15, 30, 45, 60, 180]  # degrees
path_length = 1.05  # meters

paths = [
    create_straight_line_path_from_angle(angle, path_length) for angle in path_angles
]

class OMPath(Node):
    def __init__(self):
        super().__init__("om_path")

        self.half_width_robot = 0.20
        self.sensor_mounting_angle = 172.0
        self.relevant_distance_min = 0.20

        self.obstacle_threshold = 0.50  # 50 data points

        self.scan = None
        self.obstacle = None

        self.scan_info = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            10,
        )

        self.obstacle_info = self.create_subscription(
            PointCloud,
            "/camera/realsense2_camera_node/depth/obstacle_point",
            self.obstacle_callback,
            10
        )

        self.paths_pub = self.create_publisher(
            Paths,
            "/om/paths",
            10
        )

        self.get_logger().info("OMPath node started")

    def scan_callback(self, msg: LaserScan):
        self.scan = msg

        angles = list(
            map(
                lambda x: 360.0 * (x + math.pi) / (2 * math.pi),
                np.arange(self.scan.angle_min, self.scan.angle_max, self.scan.angle_increment),
            )
        )
        angles_final = np.flip(angles)
        # angles now run from 360.0 to 0 degress
        data = list(zip(angles_final, self.scan.ranges))
        array_ready = np.array(data)

        complexes = []

        for angle, distance in data:

            d_m = distance

            # don't worry about distant objects
            if d_m > 5.0:
                continue

            # first, correctly orient the sensor zero to the robot zero
            angle = angle + self.sensor_mounting_angle
            if angle >= 360.0:
                angle = angle - 360.0
            elif angle < 0.0:
                angle = 360.0 + angle

            # then, convert to radians
            a_rad = angle * math.pi / 180.0

            v1 = d_m * math.cos(a_rad)
            v2 = d_m * math.sin(a_rad)

            # convert to x and y
            # x runs backwards to forwards, y runs left to right
            x = -1 * v2
            y = -1 * v1

            # convert the angle to -180 to + 180 range
            angle = angle - 180.0

            keep = True

            if d_m < self.relevant_distance_min:
                # this is too close, disregard
                keep = False

            # the final data ready to use for path planning
            if keep:
                complexes.append([x, y, angle, d_m])

        if self.obstacle and len(self.obstacle.points) > self.obstacle_threshold:
            for point in self.obstacle.points:
                x = point.x
                y = point.y
                z = point.z

                angle, distance = self.calculate_angle_and_distance(x, y)
                complexes.append([x, y, angle, distance])

        array = np.array(complexes)

        sorted_indices = array[:, 2].argsort()
        array = array[sorted_indices]

        X = array[:, 0]
        Y = array[:, 1]
        # A = array[:, 2]
        D = array[:, 3]

        possible_paths = np.array([0, 1, 2, 3, 4, 5, 6, 7, 8, 9])
        bad_paths = []

        for x, y, d in list(zip(X, Y, D)):
            for apath in possible_paths:
                # Get the start and end points of this straight line path
                path_points = paths[apath]
                start_x, start_y = (
                    path_points[0][0],
                    path_points[1][0],
                )
                end_x, end_y = path_points[0][-1], path_points[1][-1]

                if apath == 9:
                    # For going back, only consider obstacles that are:
                    # 1. Behind the robot (negative x in robot frame)
                    # 2. Within a certain distance threshold

                    # Assuming robot faces positive x direction
                    # Only check obstacles behind the robot
                    if y >= 0:  # Skip obstacles in front of or beside the robot
                        continue

                # Calculate distance from obstacle to the line segment
                dist_to_line = self.distance_point_to_line_segment(
                    x, y, start_x, start_y, end_x, end_y
                )

                if dist_to_line < self.half_width_robot:
                    # too close - this path will not work
                    path_to_remove = np.array([apath])
                    bad_paths.append(apath)
                    # Remove this path from the possible paths
                    # np.setdiff1d returns a new array, so we need to update possible_paths
                    possible_paths = np.setdiff1d(possible_paths, path_to_remove)
                    break  # no need to keep checking this path - we know this path is bad

        paths_msg = Paths()
        paths_msg.header.stamp = self.get_clock().now().to_msg()
        paths_msg.header.frame_id = "laser"
        paths_msg.paths = possible_paths.tolist()

        self.paths_pub.publish(paths_msg)

    def obstacle_callback(self, msg: PointCloud):
        self.obstacle = msg

    def calculate_angle_and_distance(self, world_x, world_y):
        distance = math.sqrt(world_x**2 + world_y**2)

        angle_rad = math.atan2(world_y, world_x)
        angle_degrees = math.degrees(angle_rad)

        return angle_degrees, distance

    def distance_point_to_line_segment(self, px, py, x1, y1, x2, y2):
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
