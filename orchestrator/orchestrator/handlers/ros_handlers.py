import json
import requests
import threading
from typing import TYPE_CHECKING
from uuid import uuid4
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from om_api.msg import OMAPIRequest, OMAPIResponse, MapStorage

if TYPE_CHECKING:
    from ..core.orchestrator_api import OrchestratorAPI


class ROSHandlers:
    """
    Handles ROS message callbacks and processing.
    """

    def __init__(self, orchestrator: 'OrchestratorAPI'):
        """
        Initialize the ROS handlers.

        Parameters:
        -----------
        orchestrator : OrchestratorAPI
            Reference to the main orchestrator instance.
        """
        self.orchestrator = orchestrator

    def lowstate_callback(self, msg):
        """
        Monitor BMS state for charging detection.

        Parameters:
        ----------
        msg : LowState
            The lowstate message containing BMS data.
        """
        if not hasattr(self.orchestrator, 'charging_manager'):
            self.orchestrator.get_logger().warning("Charging manager not available")
            return

        bms = msg.bms_state
        self.orchestrator.charging_manager.update_battery_state(bms.soc, bms.current)

    def ai_response_callback(self, msg):
        """
        Callback for AI response messages.

        Parameters:
        ----------
        msg : OMAIReponse
            The received AI response message.
        """
        if not hasattr(self.orchestrator, 'api_response_pub'):
            self.orchestrator.get_logger().warning("API response publisher not available")
            return

        response_msg = OMAPIResponse()
        response_msg.header.stamp = self.orchestrator.get_clock().now().to_msg()
        response_msg.request_id = msg.request_id
        response_msg.code = msg.code
        response_msg.status = msg.status
        response_msg.message = msg.status

        self.orchestrator.api_response_pub.publish(response_msg)

    def mode_response_callback(self, msg):
        """
        Callback for mode response messages.

        Parameters:
        ----------
        msg : OMModeReponse
            The received mode response message.
        """
        if not hasattr(self.orchestrator, 'api_response_pub'):
            self.orchestrator.get_logger().warning("API response publisher not available")
            return

        response_msg = OMAPIResponse()
        response_msg.header.stamp = self.orchestrator.get_clock().now().to_msg()
        response_msg.request_id = msg.request_id
        response_msg.code = msg.code
        response_msg.status = msg.current_mode
        response_msg.message = msg.message

        self.orchestrator.api_response_pub.publish(response_msg)

    def tts_response_callback(self, msg):
        """
        Callback for TTS response messages.

        Parameters:
        ----------
        msg : OMTTSReponse
            The received TTS response message.
        """
        if not hasattr(self.orchestrator, 'api_response_pub'):
            self.orchestrator.get_logger().warning("API response publisher not available")
            return

        response_msg = OMAPIResponse()
        response_msg.header.stamp = self.orchestrator.get_clock().now().to_msg()
        response_msg.request_id = msg.request_id
        response_msg.code = msg.code
        response_msg.status = msg.status
        response_msg.message = msg.status

        self.orchestrator.api_response_pub.publish(response_msg)

    def api_request_callback(self, msg):
        """
        Receive API requests from ROS2 topic and handle them.

        Parameters:
        -----------
        msg : OMAPIRequest
            The incoming API request message.
        """
        threading.Thread(target=self._process_api_request, args=(msg,)).start()

    def _process_api_request(self, msg):
        """
        Process API request in a separate thread.

        Parameters:
        -----------
        msg : OMAPIRequest
            The incoming API request message.
        """
        action = msg.action.lower()
        base_url = "http://localhost:5000"

        try:
            # Map ROS API requests to HTTP endpoints
            endpoint_map = {
                'start_slam': '/start/slam',
                'stop_slam': '/stop/slam',
                'start_nav2': '/start/nav2',
                'stop_nav2': '/stop/nav2',
                'start_base_control': '/start/base_control',
                'stop_base_control': '/stop/base_control',
                'start_charging': '/charging/dock',
                'stop_charging': '/charging/stop',
                'save_map': '/maps/save',
                'list_maps': '/maps/list',
                'delete_map': '/maps/delete',
                'add_location': '/maps/locations/add',
                'list_locations': '/maps/locations/list',
                'add_slam_location': '/maps/locations/add/slam',
                'status': '/status',
                'charging_status': '/charging/status'
            }

            if action in endpoint_map:
                endpoint = endpoint_map[action]
                url = f"{base_url}{endpoint}"

                parameters = {}
                if msg.parameters:
                    try:
                        parameters = json.loads(msg.parameters)
                    except json.JSONDecodeError:
                        parameters = {"data": msg.parameters}

                if endpoint in ['/maps/list', '/maps/locations/list', '/status', '/charging/status']:
                    response = requests.get(url, timeout=30)
                else:
                    response = requests.post(url, json=parameters, timeout=30)

                self._publish_api_response(msg.request_id, response.status_code, "success" if response.ok else "error", response.text)

            else:
                self._publish_api_response(msg.request_id, 400, "error", f"Unknown action: {action}")

        except requests.exceptions.RequestException as e:
            self._publish_api_response(msg.request_id, 500, "error", f"Request failed: {str(e)}")
        except Exception as e:
            self._publish_api_response(msg.request_id, 500, "error", f"Unexpected error: {str(e)}")

    def _publish_api_response(self, request_id: str, code: int, status: str, message: str):
        """
        Publish API response message.

        Parameters:
        -----------
        request_id : str
            The request ID to respond to.
        code : int
            HTTP status code.
        status : str
            Status string.
        message : str
            Response message.
        """
        if not hasattr(self.orchestrator, 'api_response_pub'):
            self.orchestrator.get_logger().warning("API response publisher not available")
            return

        response_msg = OMAPIResponse()
        response_msg.header.stamp = self.orchestrator.get_clock().now().to_msg()
        response_msg.request_id = request_id
        response_msg.code = code
        response_msg.status = status
        response_msg.message = message

        self.orchestrator.api_response_pub.publish(response_msg)

    def map_storage_callback(self, map_data: MapStorage):
        """
        Callback for MapStorage messages to upload maps to cloud.

        Parameters:
        -----------
        map_data : om_api.msg.MapStorage
            The incoming MapStorage message containing map details.
        """
        if hasattr(self.orchestrator, 'map_upload_manager'):
            result = self.orchestrator.map_upload_manager.upload_map(
                map_data.map_name,
                map_data.files_created,
                map_data.base_path
            )
            self.orchestrator.get_logger().info(f"Map upload result: {result}")
        else:
            self.orchestrator.get_logger().warning("Map upload manager not available")

    def cloud_pose_callback(self, pose: PoseStamped):
        """
        Callback function for PoseStamped messages to stream to cloud.

        Parameters:
        -----------
        pose : geometry_msgs.msg.PoseStamped
            The incoming PoseStamped message containing the robot's pose.
        """
        if not hasattr(self.orchestrator, 'cloud_connection_manager'):
            self.orchestrator.get_logger().warning("Cloud connection manager not available")
            return

        time = pose.header.stamp.sec + pose.header.stamp.nanosec * 1e-9
        pose_data = {
            "time": time,
            "position": {
                "x": pose.pose.position.x,
                "y": pose.pose.position.y,
                "z": pose.pose.position.z
            },
            "orientation": {
                "x": pose.pose.orientation.x,
                "y": pose.pose.orientation.y,
                "z": pose.pose.orientation.z,
                "w": pose.pose.orientation.w
            }
        }
        self.orchestrator.cloud_connection_manager.send_pose_data(pose_data)

    def cloud_map_callback(self, map_msg: OccupancyGrid):
        """
        Callback function for OccupancyGrid messages to stream to cloud.

        Parameters:
        -----------
        map_msg : nav_msgs.msg.OccupancyGrid
            The incoming OccupancyGrid message containing the map data.
        """
        if not hasattr(self.orchestrator, 'cloud_connection_manager'):
            self.orchestrator.get_logger().warning("Cloud connection manager not available")
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
        self.orchestrator.cloud_connection_manager.send_map_data(map_data)

    def cloud_api_response_callback(self, response):
        """
        Callback function for OMAPIResponse messages to send to cloud.

        Parameters:
        -----------
        response : om_api.msg.OMAPIResponse
            The incoming OMAPIResponse message containing the API response details.
        """
        if not hasattr(self.orchestrator, 'cloud_connection_manager'):
            self.orchestrator.get_logger().warning("Cloud connection manager not available")
            return

        response_data = {
            "time": response.header.stamp.sec + response.header.stamp.nanosec * 1e-9,
            "request_id": response.request_id,
            "code": response.code,
            "status": response.status,
            "message": response.message
        }
        self.orchestrator.cloud_connection_manager.send_api_response(response_data)

    def cloud_api_request_callback(self, message: str):
        """
        Callback function for messages received from the cloud API WebSocket.

        Parameters:
        -----------
        message : str
            The incoming message from the API WebSocket.
        """
        if not hasattr(self.orchestrator, 'api_request_pub'):
            self.orchestrator.get_logger().warning("API request publisher not available")
            return

        try:
            msg_json = json.loads(message)

            api_msg = OMAPIRequest()
            api_msg.header.stamp.sec = int(msg_json.get('time', 0))
            api_msg.header.stamp.nanosec = int((msg_json.get('time', 0) - api_msg.header.stamp.sec) * 1e9)
            api_msg.request_id = msg_json.get('request_id', str(uuid4()))
            api_msg.action = msg_json.get('action', '')

            parameters = msg_json.get('parameters', '')
            if isinstance(parameters, dict):
                api_msg.parameters = json.dumps(parameters)
            else:
                api_msg.parameters = str(parameters)

            if api_msg.action == '':
                self.orchestrator.get_logger().warning("Received cloud API message with missing action field.")
                return

            self.orchestrator.api_request_pub.publish(api_msg)
            self.orchestrator.get_logger().info(f"Published cloud API request: {api_msg.action} with ID {api_msg.request_id}")

        except Exception as e:
            self.orchestrator.get_logger().error(f"Failed to process cloud API message: {e}")
