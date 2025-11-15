import math
import threading
from typing import Optional

import rclpy
from action_msgs.msg import GoalStatusArray
from flask import Flask, jsonify, request
from flask_cors import CORS
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

status_map = {
    0: "UNKNOWN",
    1: "ACCEPTED",
    2: "EXECUTING",
    3: "CANCELING",
    4: "SUCCEEDED",
    5: "CANCELED",
    6: "ABORTED",
}


class G1APINode(Node):
    """
    A ROS2 node that provides a REST API for interacting with the Unitree G1 robot (RTAB-Map).
    """

    def __init__(self):
        super().__init__("g1_nav2_api_node")

        self.pose_subscription = self.create_subscription(
            PoseStamped, "/om/pose", self.pose_callback, 10
        )

        self.pose_publisher = self.create_publisher(PoseStamped, "/goal_pose", 10)

        # For Unitree G1 with RTAB-Map: subscribe to /localization_pose for localization
        self.localization_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            "/localization_pose",
            self.localization_callback,
            10,
        )

        self.nav2_status_subscription = self.create_subscription(
            GoalStatusArray,
            "/navigate_to_pose/_action/status",
            self.goal_status_callback,
            10,
        )

        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )

        self.map_subscription = self.create_subscription(
            OccupancyGrid, "/grid_prob_map", self.map_callback, map_qos
        )

        self.pose_data: Optional[PoseWithCovarianceStamped] = None
        self.nav2_status: Optional[GoalStatusArray] = None
        self.map_data: Optional[OccupancyGrid] = None

        self.app = Flask(__name__)
        CORS(self.app)
        self.register_routes()

        self.api_thread = threading.Thread(target=self.run_flask_app)
        self.api_thread.start()

    self.get_logger().info("G1 API Node initialized")

    def run_flask_app(self):
        """
        Run the Flask application in a separate thread.
        """
        self.app.run(host="0.0.0.0", port=5001, use_reloader=False)

    def register_routes(self):
        """
        Register the API routes.
        """

        @self.app.route("/api/status", methods=["GET"])
        def get_status():
            return jsonify({"status": "OK", "message": "G1 API is running"}), 200

        @self.app.route("/api/pose", methods=["GET"])
        def get_pose():
            """
            Get the current pose of the robot.
            Returns a JSON object with the robot's position and orientation.
            """
            if self.pose_data is None:
                return jsonify({"error": "Pose data not available"}), 404

            pose = {
                "position": {
                    "x": round(self.pose_data.pose.pose.position.x, 5),
                    "y": round(self.pose_data.pose.pose.position.y, 5),
                    "z": round(self.pose_data.pose.pose.position.z, 5),
                },
                "orientation": {
                    "x": round(self.pose_data.pose.pose.orientation.x, 5),
                    "y": round(self.pose_data.pose.pose.orientation.y, 5),
                    "z": round(self.pose_data.pose.pose.orientation.z, 5),
                    "w": round(self.pose_data.pose.pose.orientation.w, 5),
                },
                "covariance": self.pose_data.pose.covariance.tolist(),
            }

            return jsonify(pose), 200

        @self.app.route("/api/move_to_pose", methods=["POST"])
        def move_to_pose():
            """
            Move the robot to a specified pose.
            Expects a JSON object with 'position' and 'orientation'.
            """
            data = request.json
            if not data or "position" not in data or "orientation" not in data:
                return jsonify({"error": "Invalid input"}), 400

            position = data["position"]
            orientation = data["orientation"]

            if not all(k in position for k in ("x", "y", "z")) or not all(
                k in orientation for k in ("x", "y", "z", "w")
            ):
                return (
                    jsonify(
                        {
                            "error": "Position and orientation must contain x, y, z, and w"
                        }
                    ),
                    400,
                )

            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "map"
            pose_msg.pose.position.x = position["x"]
            pose_msg.pose.position.y = position["y"]
            pose_msg.pose.position.z = position["z"]
            pose_msg.pose.orientation.x = orientation["x"]
            pose_msg.pose.orientation.y = orientation["y"]
            pose_msg.pose.orientation.z = orientation["z"]
            pose_msg.pose.orientation.w = orientation["w"]

            self.pose_publisher.publish(pose_msg)

            self.get_logger().info(f"Moving to pose: {pose_msg.pose}")

            return (
                jsonify({"status": "success", "message": "Moving to specified pose"}),
                200,
            )

        @self.app.route("/api/amcl_variance", methods=["GET"])
        def get_amcl_variance():
            """
            Get the AMCL pose variance.
            """
            if self.pose_data is None:
                return jsonify({"error": "AMCL pose data not available"}), 404

            try:
                covariance = self.pose_data.pose.covariance
                x_uncertainty = round(covariance[0] ** (1 / 2), 5)
                y_uncertainty = round(covariance[7] ** (1 / 2), 5)
                yaw_uncertainty = round(covariance[35] ** (1 / 2) / math.pi * 180, 5)

                return (
                    jsonify(
                        {
                            "x_uncertainty": x_uncertainty,
                            "y_uncertainty": y_uncertainty,
                            "yaw_uncertainty": yaw_uncertainty,
                        }
                    ),
                    200,
                )

            except IndexError:
                return jsonify({"error": "Invalid covariance data"}), 500

        @self.app.route("/api/nav2_status", methods=["GET"])
        def get_nav2_status():
            """
            Get the status of the Nav2 stack.
            """
            if self.nav2_status is None:
                return jsonify({"error": "Nav2 status not available"}), 404

            status_list = []

            for status in self.nav2_status.status_list:
                uuid_bytes = status.goal_info.goal_id.uuid
                goal_id = "".join(f"{b:02x}" for b in uuid_bytes)

                status_info = {
                    "goal_id": goal_id,
                    "status": status_map.get(status.status, "UNKNOWN"),
                    "timestamp": {
                        "sec": status.goal_info.stamp.sec,
                        "nanosec": status.goal_info.stamp.nanosec,
                    },
                }
                status_list.append(status_info)

            return jsonify({"nav2_status": status_list}), 200

        @self.app.route("/api/map", methods=["GET"])
        def get_map():
            """
            Get the current map data.
            Returns a JSON object with the map metadata and occupancy grid.
            """
            if self.map_data is None:
                return jsonify({"error": "Map data not available"}), 404

            map_info = {
                "map_metadata": {
                    "map_load_time": self.map_data.info.map_load_time.sec
                    + self.map_data.info.map_load_time.nanosec * 1e-9,
                    "resolution": self.map_data.info.resolution,
                    "width": self.map_data.info.width,
                    "height": self.map_data.info.height,
                    "origin": {
                        "position": {
                            "x": self.map_data.info.origin.position.x,
                            "y": self.map_data.info.origin.position.y,
                            "z": self.map_data.info.origin.position.z,
                        },
                        "orientation": {
                            "x": self.map_data.info.origin.orientation.x,
                            "y": self.map_data.info.origin.orientation.y,
                            "z": self.map_data.info.origin.orientation.z,
                            "w": self.map_data.info.origin.orientation.w,
                        },
                    },
                },
                "data": list(self.map_data.data),
            }

            return jsonify(map_info), 200

    def pose_callback(self, msg: PoseStamped):
        """
        Callback function for PoseStamped messages.
        Updates the internal pose data and logs the received pose.

        Parameters:
        -----------
        msg : geometry_msgs.msg.PoseStamped
            The incoming PoseStamped message containing the robot's pose.
        """
        covariance = [0.0] * 36

        if self.pose_data is None:
            self.pose_data = PoseWithCovarianceStamped()
        else:
            covariance = self.pose_data.pose.covariance

        self.pose_data.header = msg.header
        self.pose_data.pose.pose = msg.pose
        self.pose_data.pose.covariance = covariance

    def localization_callback(self, msg: PoseWithCovarianceStamped):
        """
        Callback for Unitree G1 localization updates from RTAB-Map (/localization_pose).
        Updates the internal pose data and logs the received localization pose.

        Parameters:
        -----------
        msg : geometry_msgs.msg.PoseWithCovarianceStamped
            The incoming localization pose message containing the robot's pose with covariance.
        """
        if self.pose_data is None:
            self.pose_data = PoseWithCovarianceStamped()
        self.pose_data.pose.covariance = msg.pose.covariance
        self.get_logger().info(
            "[G1] Updated localization covariance from /localization_pose."
        )

    def goal_status_callback(self, msg: GoalStatusArray):
        """
        Callback function for Nav2 goal status updates.
        Updates the internal Nav2 status data and logs the received status.

        Parameters:
        -----------
        msg : action_msgs.msg.GoalStatusArray
            The incoming goal status message containing the status of active goals.
        """
        self.nav2_status = msg

    def map_callback(self, msg: OccupancyGrid):
        """
        Callback function for OccupancyGrid messages.
        Updates the internal map data and logs the received map.

        Parameters:
        -----------
        msg : nav_msgs.msg.OccupancyGrid
            The incoming OccupancyGrid message containing the map data.
        """
        self.map_data = msg
        self.get_logger().info(
            f"Received map with dimensions: {msg.info.width}x{msg.info.height}"
        )


def main(args=None):
    """
    Main function to initialize the ROS2 node and start the G1 API.
    """
    rclpy.init(args=args)

    g1_api_node = G1APINode()

    try:
        rclpy.spin(g1_api_node)
    except KeyboardInterrupt:
        pass
    finally:
        g1_api_node.destroy_node()
        rclpy.shutdown()
