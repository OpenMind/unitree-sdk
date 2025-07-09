import rclpy
import math
from rclpy.node import Node
from flask import Flask, jsonify, request
from flask_cors import CORS
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import threading
from typing import Optional
from action_msgs.msg import GoalStatusArray

status_map = {
    0: "UNKNOWN",
    1: "ACCEPTED",
    2: "EXECUTING",
    3: "CANCELING",
    4: "SUCCEEDED",
    5: "CANCELED",
    6: "ABORTED"
}

class Go2APINode(Node):
    """
    A ROS2 node that provides a REST API for interacting with the Unitree Go2 robot.
    """
    def __init__(self):
        super().__init__("go2_api_node")

        self.pose_subscription = self.create_subscription(
            PoseStamped,
            '/om/pose',
            self.pose_callback,
            10
        )

        self.pose_publisher = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10
        )

        self.amcl_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_callback,
            10
        )

        self.nav2_status_subscription = self.create_subscription(
            GoalStatusArray,
            '/navigate_to_pose/_action/status',
            self.goal_status_callback,
            10
        )

        self.pose_data: Optional[PoseWithCovarianceStamped] = None
        self.nav2_status: Optional[GoalStatusArray] = None

        self.app = Flask(__name__)
        CORS(self.app)
        self.register_routes()

        self.api_thread = threading.Thread(target=self.run_flask_app)
        self.api_thread.start()

        self.get_logger().info("Go2 API Node initialized")

    def run_flask_app(self):
        """
        Run the Flask application in a separate thread.
        """
        self.app.run(host='0.0.0.0', port=5000, use_reloader=False)

    def register_routes(self):
        """
        Register the API routes.
        """
        @self.app.route('/api/status', methods=['GET'])
        def get_status():
            return jsonify({"status": "OK", "message": "Go2 API is running"}), 200

        @self.app.route('/api/pose', methods=['GET'])
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
                "covariance": self.pose_data.pose.covariance.tolist()
            }

            return jsonify(pose), 200

        @self.app.route('/api/move_to_pose', methods=['POST'])
        def move_to_pose():
            """
            Move the robot to a specified pose.
            Expects a JSON object with 'position' and 'orientation'.
            """
            data = request.json
            if not data or 'position' not in data or 'orientation' not in data:
                return jsonify({"error": "Invalid input"}), 400

            position = data['position']
            orientation = data['orientation']

            if not all(k in position for k in ('x', 'y', 'z')) or not all(k in orientation for k in ('x', 'y', 'z', 'w')):
                return jsonify({"error": "Position and orientation must contain x, y, z, and w"}), 400

            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'map'
            pose_msg.pose.position.x = position['x']
            pose_msg.pose.position.y = position['y']
            pose_msg.pose.position.z = position['z']
            pose_msg.pose.orientation.x = orientation['x']
            pose_msg.pose.orientation.y = orientation['y']
            pose_msg.pose.orientation.z = orientation['z']
            pose_msg.pose.orientation.w = orientation['w']

            self.pose_publisher.publish(pose_msg)

            self.get_logger().info(f"Moving to pose: {pose_msg.pose}")

            return jsonify({"status": "success", "message": "Moving to specified pose"}), 200

        @self.app.route('/api/amcl_variance', methods=['GET'])
        def get_amcl_variance():
            """
            Get the AMCL pose variance.
            """
            if self.pose_data is None:
                return jsonify({"error": "AMCL pose data not available"}), 404

            try:
                covariance = self.pose_data.pose.covariance
                x_uncertainty = round(covariance[0] ** (1/2), 5)
                y_uncertainty = round(covariance[7] ** (1/2), 5)
                yaw_uncertainty = round(covariance[35] ** (1/2) / math.pi * 180, 5)

                return jsonify({
                    "x_uncertainty": x_uncertainty,
                    "y_uncertainty": y_uncertainty,
                    "yaw_uncertainty": yaw_uncertainty
                }), 200

            except IndexError:
                return jsonify({"error": "Invalid covariance data"}), 500

        @self.app.route('/api/nav2_status', methods=['GET'])
        def get_nav2_status():
            """
            Get the status of the Nav2 stack.
            """
            if self.nav2_status is None:
                return jsonify({"error": "Nav2 status not available"}), 404

            status_list = []

            for status in self.nav2_status.status_list:
                uuid_bytes = status.goal_info.goal_id.uuid
                goal_id = ''.join(f'{b:02x}' for b in uuid_bytes)

                status_info = {
                    "goal_id": goal_id,
                    "status": status_map.get(status.status, "UNKNOWN"),
                    "timestamp": {
                        "sec": status.goal_info.stamp.sec,
                        "nanosec": status.goal_info.stamp.nanosec
                    }
                }
                status_list.append(status_info)

            return jsonify({"nav2_status": status_list}), 200

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

    def amcl_callback(self, msg: PoseWithCovarianceStamped):
        """
        Callback function for AMCL pose updates.
        Updates the internal pose data and logs the received AMCL pose.

        Parameters:
        -----------
        msg : geometry_msgs.msg.PoseWithCovarianceStamped
            The incoming AMCL pose message containing the robot's pose with covariance.
        """
        if self.pose_data is None:
            self.pose_data = PoseWithCovarianceStamped()
        self.pose_data.pose.covariance = msg.pose.covariance

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

def main(args=None):
    """
    Main function to initialize the ROS2 node and start the API.
    """
    rclpy.init(args=args)

    go2_api_node = Go2APINode()

    try:
        rclpy.spin(go2_api_node)
    except KeyboardInterrupt:
        pass
    finally:
        go2_api_node.destroy_node()
        rclpy.shutdown()
