import os
import requests
import rclpy
from rclpy.node import Node
from uuid import uuid4
from om_api.msg import MapStorage, OMAPIRequest, OMAPIResponse
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
import json

from .ws_client import WebSocketClient

map_api_url = "https://api.openmind.org/api/core/maps/upload"
pose_ws_url = "wss://api.openmind.org/api/core/teleops/pose"
map_ws_url = "wss://api.openmind.org/api/core/teleops/maps"
api_ws_url = "wss://api.openmind.org/api/core/teleops/api"

class OrchestratorCloud(Node):
    """
    Orchestrator for managing cloud-related tasks.
    """
    def __init__(self):
        super().__init__('orchestrator_cloud')

        self.api_key = os.getenv('OM_API_KEY')
        if not self.api_key:
            self.get_logger().error("OM_API_KEY environment variable not set!")

        self.map_saver_sub = self.create_subscription(
            MapStorage,
            '/om/map_storage',
            self.upload_map,
            10,
        )

        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/om/pose',
            self.pose_callback,
            10,
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10,
        )

        self.api_request_pub = self.create_publisher(
            OMAPIRequest,
            '/om/api/request',
            10,
        )

        self.api_response_sub = self.create_subscription(
            OMAPIResponse,
            '/om/api/response',
            self.api_response_callback,
            10,
        )

        self.pose_ws_url = f"{pose_ws_url}?api_key={self.api_key}" if self.api_key else None
        self.map_ws_url = f"{map_ws_url}?api_key={self.api_key}" if self.api_key else None
        self.api_ws_url = f"{api_ws_url}?api_key={self.api_key}" if self.api_key else None

        self.pose_ws = WebSocketClient(self.pose_ws_url, self.get_logger()) if self.pose_ws_url else None
        self.map_ws = WebSocketClient(self.map_ws_url, self.get_logger()) if self.map_ws_url else None
        self.api_ws = WebSocketClient(self.api_ws_url, self.get_logger()) if self.api_ws_url else None

        if self.pose_ws:
            self.pose_ws.start()

        if self.map_ws:
            self.map_ws.start()

        if self.api_ws:
            self.api_ws.start()
            self.api_ws.message_callback = self.api_request_callback

        self.get_logger().info("Orchestrator Cloud Node Initialized")

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
            self.get_logger().error("Cannot upload map: OM_API_KEY not set.")
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
            self.get_logger().error("Cannot send pose: OM_API_KEY not set.")
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

        if self.pose_ws and self.pose_ws.connected:
            try:
                self.pose_ws.send_message(json.dumps(pose_data))
                self.get_logger().info("Pose data sent.")
            except Exception as e:
                self.get_logger().error(f"Failed to send pose data: {e}")

    def map_callback(self, map_msg: OccupancyGrid):
        """
        Callback function for OccupancyGrid messages.

        Parameters:
        -----------
        map_msg : nav_msgs.msg.OccupancyGrid
            The incoming OccupancyGrid message containing the map data.
        """
        if not self.api_key:
            self.get_logger().error("Cannot send map: OM_API_KEY not set.")
            return

        map_data = {
            "header": {
                "stamp": {
                    "sec": map_msg.header.stamp.sec,
                    "nanosec": map_msg.header.stamp.nanosec
                },
                "frame_id": map_msg.header.frame_id
            },
            "info": {
                "resolution": map_msg.info.resolution,
                "width": map_msg.info.width,
                "height": map_msg.info.height,
                "origin": {
                    "position": {
                        "x": map_msg.info.origin.position.x,
                        "y": map_msg.info.origin.position.y,
                        "z": map_msg.info.origin.position.z
                    },
                    "orientation": {
                        "x": map_msg.info.origin.orientation.x,
                        "y": map_msg.info.origin.orientation.y,
                        "z": map_msg.info.origin.orientation.z,
                        "w": map_msg.info.origin.orientation.w
                    }
                }
            },
            "data": list(map_msg.data)
        }

        if self.map_ws and self.map_ws.connected:
            try:
                self.map_ws.send_message(json.dumps(map_data))
                self.get_logger().info("Map data sent.")
            except Exception as e:
                self.get_logger().error(f"Failed to send map data: {e}")

    def api_request_callback(self, message: str):
        """
        Callback function for messages received from the API WebSocket.

        Parameters:
        -----------
        message : str
            The incoming message from the API WebSocket.
        """
        try:
            msg_json = json.loads(message)
            api_msg = OMAPIRequest()
            api_msg.header.stamp.sec = int(msg_json.get('time', 0))
            api_msg.header.stamp.nanosec = int((msg_json.get('time', 0) - api_msg.header.stamp.sec) * 1e9)
            api_msg.request_id = msg_json.get('request_id', str(uuid4()))
            api_msg.action = msg_json.get('action', '')
            api_msg.status = msg_json.get('status', '')
            api_msg.parameters = msg_json.get('parameters', '')

            if msg_json.action == '' or msg_json.status == '':
                self.get_logger().warning("Received API message with missing fields.")
                return

            self.api_request_pub.publish(api_msg)
            self.get_logger().info(f"Published API message: action={api_msg.action}, status={api_msg.status}")
        except Exception as e:
            self.get_logger().error(f"Failed to process API message: {e}")

    def api_response_callback(self, response: OMAPIResponse):
        """
        Callback function for OMAPIResponse messages.

        Parameters:
        -----------
        response : om_api.msg.OMAPIResponse
            The incoming OMAPIResponse message containing the API response details.
        """
        if not self.api_key:
            self.get_logger().error("Cannot send API response: OM_API_KEY not set.")
            return

        response_data = {
            "time": response.header.stamp.sec + response.header.stamp.nanosec * 1e-9,
            "request_id": response.request_id,
            "code": response.code,
            "status": response.status,
            "responses": response.responses
        }

        if self.api_ws and self.api_ws.connected:
            try:
                self.api_ws.send_message(json.dumps(response_data))
                self.get_logger().info("API response sent.")
            except Exception as e:
                self.get_logger().error(f"Failed to send API response: {e}")

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

        if orchestrator_cloud.pose_ws:
            orchestrator_cloud.pose_ws.stop()
        if orchestrator_cloud.map_ws:
            orchestrator_cloud.map_ws.stop()

        rclpy.shutdown()

if __name__ == '__main__':
    main()

