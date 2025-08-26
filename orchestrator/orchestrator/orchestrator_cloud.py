import os
import requests
import rclpy
from rclpy.node import Node
from om_api.msg import MapStorage
from geometry_msgs.msg import PoseStamped
import websocket
import json

map_api_url = "https://api.openmind.org/api/core/maps/upload"
pose_ws_url = "wss://api.openmind.org/api/core/teleops/pose"

class OrchestratorCloud(Node):
    """
    Orchestrator for managing cloud-related tasks.
    """
    def __init__(self):
        super().__init__('orchestrator_cloud')

        self.api_key = os.getenv('OPENMIND_API_KEY')
        if not self.api_key:
            self.get_logger().error("OPENMIND_API_KEY environment variable not set!")

        self.map_saver_sub = self.create_subscription(
            MapStorage,
            '/om/map_storage',
            self.upload_map,
            10
        )

        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/om/pose',
            self.pose_callback,
            10
        )

        self.ws = None
        self.connect_websocket()

        self.get_logger().info("Orchestrator Cloud Node Initialized")

    def connect_websocket(self):
        """
        Connect to the WebSocket server for sending pose data.
        """
        if not self.api_key:
            self.get_logger().error("Cannot connect to WebSocket: OPENMIND_API_KEY not set.")
            return

        try:
            ws_url = f"{pose_ws_url}?api_key={self.api_key}"
            self.ws = websocket.create_connection(ws_url)
            self.get_logger().info("WebSocket connected")
        except Exception as e:
            self.get_logger().error(f"WebSocket connection failed: {e}")
            self.ws = None

    def upload_map(self, map_data: MapStorage):
        """
        Upload map data to the cloud

        Parameters:
        -----------
        map_data : om_api.msg.MapStorage
            The incoming MapStorage message containing map details.
        """
        map_name = map_data.map_name
        files_created = map_data.files_created
        base_path = map_data.base_path

        self.get_logger().info(f"Uploading map: {map_name}")
        self.get_logger().info(f"Files to upload: {files_created}")
        self.get_logger().info(f"Base path: {base_path}")

        if not self.api_key:
            self.get_logger().error("Cannot upload map: OPENMIND_API_KEY not set.")
            return

        if "yaml" and "pgm" not in files_created:
            self.get_logger().error("Map files missing: both .yaml and .pgm files are required.")
            return

        yaml_path = f"{base_path}.yaml"
        pgm_path = f"{base_path}.pgm"

        if not os.path.exists(yaml_path) or not os.path.exists(pgm_path):
            self.get_logger().error("Map files do not exist at the specified paths.")
            return

        with open(yaml_path, 'rb') as yaml_file, open(pgm_path, 'rb') as pgm_file:
            files = {
                'map_yaml': (f"{map_name}.yaml", yaml_file, 'application/x-yaml'),
                'map_pgm': (f"{map_name}.pgm", pgm_file, 'image/x-portable-graymap')
            }
            headers = {
                'Authorization': f'Bearer {self.api_key}'
            }

            self.get_logger().info(f"Uploading map '{map_name}' to {map_api_url}... with headers {headers}")
            try:
                response = requests.put(map_api_url, files=files, headers=headers, data={'map_name': map_name})
                response.raise_for_status()
                self.get_logger().info(f"Map '{map_name}' uploaded successfully.")
            except Exception as e:
                self.get_logger().error(f"Failed to upload map '{map_name}': {response.text}")

    def pose_callback(self, pose: PoseStamped):
        """
        Callback function for PoseStamped messages.

        Parameters:
        -----------
        pose : geometry_msgs.msg.PoseStamped
            The incoming PoseStamped message containing the robot's pose.
        """
        time = pose.header.stamp.sec + pose.header.stamp.nanosec * 1e-9
        pose_x = pose.pose.position.x
        pose_y = pose.pose.position.y
        pose_z = pose.pose.position.z
        orientation_x = pose.pose.orientation.x
        orientation_y = pose.pose.orientation.y
        orientation_z = pose.pose.orientation.z
        orientation_w = pose.pose.orientation.w

        if not self.api_key:
            self.get_logger().error("Cannot send pose: OPENMIND_API_KEY not set.")
            return

        pose_data = {
            "time": time,
            "position": {
                "x": pose_x,
                "y": pose_y,
                "z": pose_z
            },
            "orientation": {
                "x": orientation_x,
                "y": orientation_y,
                "z": orientation_z,
                "w": orientation_w
            }
        }

        if self.ws is None:
            self.get_logger().info("WebSocket not connected, attempting to reconnect...")
            self.connect_websocket()

        if self.ws:
            try:
                self.ws.send(json.dumps(pose_data))
                self.get_logger().info(f"Pose sent: {pose_data}")
            except Exception as e:
                self.get_logger().error(f"Failed to send pose data: {e}")
                self.ws = None

def main(args=None):
    """
    Main function to run the Orchestrator cloud node
    """
    rclpy.init(args=args)

    orchestrator_cloud = OrchestratorCloud()

    try:
        rclpy.spin(orchestrator_cloud)
    except KeyboardInterrupt:
        pass
    finally:
        orchestrator_cloud.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

