import json
import threading
from typing import TYPE_CHECKING
from uuid import uuid4

import requests
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid

from om_api.msg import (
    MapStorage,
    OMAIRequest,
    OMAPIRequest,
    OMAPIResponse,
    OMASRText,
    OMAvatarFaceRequest,
    OMModeRequest,
    OMTTSRequest,
)

if TYPE_CHECKING:
    from ..core.orchestrator_api import OrchestratorAPI


class ROSHandlers:
    """
    Handles ROS message callbacks and processing.
    """

    def __init__(self, orchestrator: "OrchestratorAPI"):
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
        if not hasattr(self.orchestrator, "charging_manager"):
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
        if not hasattr(self.orchestrator, "api_response_pub"):
            self.orchestrator.get_logger().warning(
                "API response publisher not available"
            )
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
        if not hasattr(self.orchestrator, "api_response_pub"):
            self.orchestrator.get_logger().warning(
                "API response publisher not available"
            )
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
        if not hasattr(self.orchestrator, "api_response_pub"):
            self.orchestrator.get_logger().warning(
                "API response publisher not available"
            )
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
            # Handle HTTP-based endpoints
            endpoint_map = {
                "start_slam": "/start/slam",
                "stop_slam": "/stop/slam",
                "start_nav2": "/start/nav2",
                "stop_nav2": "/stop/nav2",
                "start_base_control": "/start/base_control",
                "stop_base_control": "/stop/base_control",
                "start_charging": "/charging/dock",
                "stop_charging": "/charging/stop",
                "save_map": "/maps/save",
                "list_maps": "/maps/list",
                "delete_map": "/maps/delete",
                "add_location": "/maps/locations/add",
                "list_locations": "/maps/locations/list",
                "add_slam_location": "/maps/locations/add/slam",
                "status": "/status",
                "charging_status": "/charging/status",
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

                if endpoint in [
                    "/maps/list",
                    "/maps/locations/list",
                    "/status",
                    "/charging/status",
                ]:
                    response = requests.get(url, timeout=30)
                else:
                    response = requests.post(url, json=parameters, timeout=30)

                self._publish_api_response(
                    msg.request_id,
                    response.status_code,
                    "success" if response.ok else "error",
                    response.text,
                )

            # Handle AI-related actions
            elif action in ["ai_status", "enable_ai", "disable_ai"]:
                self._handle_ai_request(action, msg)

            # Handle mode-related actions
            elif action in ["get_mode", "switch_mode"]:
                self._handle_mode_request(action, msg)

            # Handle avatar-healthcheck actions
            elif action in ["get_avatar_status"]:
                self._handle_avatar_request(action, msg)

            # Handle TTS-related actions
            elif action in ["tts_status", "enable_tts", "disable_tts"]:
                self._handle_tts_request(action, msg)

            # Handle remote control action
            elif action == "remote_control":
                self._handle_remote_control(msg)

            else:
                self.orchestrator.get_logger().error(f"Unknown action: {action}")
                self._publish_api_response(
                    msg.request_id, 400, "error", f"Unknown action: {action}"
                )

        except requests.exceptions.RequestException as e:
            self.orchestrator.get_logger().error(f"HTTP request failed: {str(e)}")
            self._publish_api_response(
                msg.request_id, 500, "error", f"Request failed: {str(e)}"
            )
        except Exception as e:
            self.orchestrator.get_logger().error(
                f"Unexpected error processing API request: {str(e)}"
            )
            self._publish_api_response(
                msg.request_id, 500, "error", f"Unexpected error: {str(e)}"
            )

    def _handle_ai_request(self, action: str, msg):
        """
        Handle AI-related requests.

        Parameters:
        -----------
        action : str
            The AI action to perform (ai_status, enable_ai, disable_ai).
        msg : OMAPIRequest
            The original API request message.
        """
        action_codes = {
            "ai_status": 2,  # Status request
            "enable_ai": 1,  # Enable AI
            "disable_ai": 0,  # Disable AI
        }

        if action not in action_codes:
            self._publish_api_response(
                msg.request_id, 400, "error", f"Unknown AI action: {action}"
            )
            return

        self.orchestrator.get_logger().info(f"Received request for {action}")

        if not hasattr(self.orchestrator, "ai_request_pub"):
            self._publish_api_response(
                msg.request_id, 500, "error", "AI request publisher not available"
            )
            return

        try:
            ai_request_msg = OMAIRequest()
            ai_request_msg.header.stamp = self.orchestrator.get_clock().now().to_msg()
            ai_request_msg.header.frame_id = "om_api"
            ai_request_msg.request_id = msg.request_id
            ai_request_msg.code = action_codes[action]

            self.orchestrator.ai_request_pub.publish(ai_request_msg)
            self.orchestrator.get_logger().info(f"Published AI request: {action}")

        except Exception as e:
            self.orchestrator.get_logger().error(
                f"Failed to publish AI request: {str(e)}"
            )
            self._publish_api_response(
                msg.request_id, 500, "error", f"Failed to publish AI request: {str(e)}"
            )

    def _handle_mode_request(self, action: str, msg):
        """
        Handle mode-related requests.

        Parameters:
        -----------
        action : str
            The mode action to perform (get_mode, switch_mode).
        msg : OMAPIRequest
            The original API request message.
        """
        if action == "switch_mode" and not msg.parameters:
            self._publish_api_response(
                msg.request_id,
                400,
                "error",
                "Mode parameter is required for switch_mode action",
            )
            return

        action_codes = {"get_mode": 1, "switch_mode": 0}  # Get current mode  # Set mode

        if action not in action_codes:
            self._publish_api_response(
                msg.request_id, 400, "error", f"Unknown mode action: {action}"
            )
            return

        self.orchestrator.get_logger().info(f"Received request for {action}")

        if not hasattr(self.orchestrator, "mode_request_pub"):
            self._publish_api_response(
                msg.request_id, 500, "error", "Mode request publisher not available"
            )
            return

        try:
            mode_request_msg = OMModeRequest()
            mode_request_msg.header.stamp = self.orchestrator.get_clock().now().to_msg()
            mode_request_msg.header.frame_id = "om_api"
            mode_request_msg.request_id = msg.request_id
            mode_request_msg.code = action_codes[action]
            mode_request_msg.mode = msg.parameters if action == "switch_mode" else ""

            self.orchestrator.mode_request_pub.publish(mode_request_msg)
            self.orchestrator.get_logger().info(f"Published mode request: {action}")

        except Exception as e:
            self.orchestrator.get_logger().error(
                f"Failed to publish mode request: {str(e)}"
            )
            self._publish_api_response(
                msg.request_id,
                500,
                "error",
                f"Failed to publish mode request: {str(e)}",
            )

    def _handle_avatar_request(self, action: str, msg):
        """
        Handle avatar-related requests.

        Parameters:
        -----------
        action : str
            The health check action to perform on avatar.
        msg : OMAPIRequest
            The original API request message.
        """
        action_codes = {"get_avatar_status": 1} 

        if action not in action_codes:
            self._publish_api_response(
                msg.request_id, 400, "error", f"Unknown avatar action: {action}"
            )
            return

        self.orchestrator.get_logger().info(f"Received request for {action}")

        if not hasattr(self.orchestrator, "avatar_request_pub"):
            self._publish_api_response(
                msg.request_id, 500, "error", "Avatar request publisher not available"
            )
            return

        try:
            avatar_request_msg = OMAvatarFaceRequest()
            avatar_request_msg.header.stamp = self.orchestrator.get_clock().now().to_msg()
            avatar_request_msg.header.frame_id = "om_api"
            avatar_request_msg.request_id = msg.request_id
            avatar_request_msg.face_text = "health_check"

            self.orchestrator.avatar_request_pub.publish(avatar_request_msg)
            self.orchestrator.get_logger().info(f"Published avatar request: {action}")

        except Exception as e:
            self.orchestrator.get_logger().error(
                f"Failed to publish avatar request: {str(e)}"
            )
            self._publish_api_response(
                msg.request_id,
                500,
                "error",
                f"Failed to publish avatar request: {str(e)}",
            )

    def _handle_tts_request(self, action: str, msg):
        """
        Handle TTS-related requests.

        Parameters:
        -----------
        action : str
            The TTS action to perform (tts_status, enable_tts, disable_tts).
        msg : OMAPIRequest
            The original API request message.
        """
        action_codes = {
            "tts_status": 2,  # Status request
            "enable_tts": 1,  # Enable TTS
            "disable_tts": 0,  # Disable TTS
        }

        if action not in action_codes:
            self._publish_api_response(
                msg.request_id, 400, "error", f"Unknown TTS action: {action}"
            )
            return

        self.orchestrator.get_logger().info(f"Received request for {action}")

        if not hasattr(self.orchestrator, "tts_request_pub"):
            self._publish_api_response(
                msg.request_id, 500, "error", "TTS request publisher not available"
            )
            return

        try:
            tts_request_msg = OMTTSRequest()
            tts_request_msg.header.stamp = self.orchestrator.get_clock().now().to_msg()
            tts_request_msg.header.frame_id = "om_api"
            tts_request_msg.request_id = msg.request_id
            tts_request_msg.code = action_codes[action]

            self.orchestrator.tts_request_pub.publish(tts_request_msg)
            self.orchestrator.get_logger().info(f"Published TTS request: {action}")

        except Exception as e:
            self.orchestrator.get_logger().error(
                f"Failed to publish TTS request: {str(e)}"
            )
            self._publish_api_response(
                msg.request_id, 500, "error", f"Failed to publish TTS request: {str(e)}"
            )

    def _handle_remote_control(self, msg):
        """
        Handle remote control requests.

        Parameters:
        -----------
        msg : OMAPIRequest
            The original API request message containing movement parameters.
        """
        if not msg.parameters:
            self._publish_api_response(
                msg.request_id,
                400,
                "error",
                "Command parameter is required for remote_control action",
            )
            return

        try:
            speed_parameters = json.loads(msg.parameters)
        except json.JSONDecodeError:
            self._publish_api_response(
                msg.request_id, 400, "error", "Invalid JSON in parameters"
            )
            return

        vx = speed_parameters.get("vx", 0.0)
        vy = speed_parameters.get("vy", 0.0)
        vyaw = speed_parameters.get("vyaw", 0.0)

        if not hasattr(self.orchestrator, "move_cmd_pub"):
            self._publish_api_response(
                msg.request_id, 500, "error", "Move command publisher not available"
            )
            return

        try:
            twist = Twist()
            twist.linear.x = float(vx)
            twist.linear.y = float(vy)
            twist.angular.z = float(vyaw)

            self.orchestrator.move_cmd_pub.publish(twist)
            self.orchestrator.get_logger().info(
                f"Published remote control command: vx={vx}, vy={vy}, vyaw={vyaw}"
            )

            self._publish_api_response(
                msg.request_id,
                200,
                "success",
                f"Remote control command published: vx={vx}, vy={vy}, vyaw={vyaw}",
            )

        except Exception as e:
            self.orchestrator.get_logger().error(
                f"Failed to publish remote control command: {str(e)}"
            )
            self._publish_api_response(
                msg.request_id,
                500,
                "error",
                f"Failed to publish remote control command: {str(e)}",
            )

    def _publish_api_response(
        self, request_id: str, code: int, status: str, message: str
    ):
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
        if not hasattr(self.orchestrator, "api_response_pub"):
            self.orchestrator.get_logger().warning(
                "API response publisher not available"
            )
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
        if hasattr(self.orchestrator, "map_upload_manager"):
            result = self.orchestrator.map_upload_manager.upload_map(
                map_data.map_name, map_data.files_created, map_data.base_path
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
        if not hasattr(self.orchestrator, "cloud_connection_manager"):
            self.orchestrator.get_logger().warning(
                "Cloud connection manager not available"
            )
            return

        time = pose.header.stamp.sec + pose.header.stamp.nanosec * 1e-9
        pose_data = {
            "time": time,
            "position": {
                "x": pose.pose.position.x,
                "y": pose.pose.position.y,
                "z": pose.pose.position.z,
            },
            "orientation": {
                "x": pose.pose.orientation.x,
                "y": pose.pose.orientation.y,
                "z": pose.pose.orientation.z,
                "w": pose.pose.orientation.w,
            },
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
        if not hasattr(self.orchestrator, "cloud_connection_manager"):
            self.orchestrator.get_logger().warning(
                "Cloud connection manager not available"
            )
            return

        map_data = {
            "header": {
                "stamp": {
                    "sec": map_msg.header.stamp.sec,
                    "nanosec": map_msg.header.stamp.nanosec,
                },
                "frame_id": map_msg.header.frame_id,
            },
            "info": {
                "resolution": map_msg.info.resolution,
                "width": map_msg.info.width,
                "height": map_msg.info.height,
                "origin": {
                    "position": {
                        "x": map_msg.info.origin.position.x,
                        "y": map_msg.info.origin.position.y,
                        "z": map_msg.info.origin.position.z,
                    },
                    "orientation": {
                        "x": map_msg.info.origin.orientation.x,
                        "y": map_msg.info.origin.orientation.y,
                        "z": map_msg.info.origin.orientation.z,
                        "w": map_msg.info.origin.orientation.w,
                    },
                },
            },
            "data": list(map_msg.data),
        }
        self.orchestrator.cloud_connection_manager.send_map_data(map_data)

    def asr_text_callback(self, msg: OMASRText):
        """
        Callback function for ASR text messages to broadcast to frontend.

        Parameters:
        -----------
        msg : OMASRText
            The incoming ASR text message.
        """
        if not hasattr(self.orchestrator, "cloud_connection_manager"):
            self.orchestrator.get_logger().warning(
                "Cloud connection manager not available"
            )
            return

        # Broadcast ASR text to WebSocket clients
        asr_data = {
            "type": "asr",
            "text": msg.text,
            "timestamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
        }

        if self.orchestrator.cloud_connection_manager.local_api_ws:
            try:
                self.orchestrator.cloud_connection_manager.local_api_ws.broadcast(
                    json.dumps(asr_data)
                )
                self.orchestrator.get_logger().info(
                    f"Broadcasted ASR text to frontend: {msg.text}"
                )
            except Exception as e:
                self.orchestrator.get_logger().error(
                    f"Failed to broadcast ASR text: {e}"
                )

    def avatar_request_callback(self, msg: OMAvatarFaceRequest):
        """
        Callback function for avatar request messages to broadcast to frontend.

        Parameters:
        -----------
        msg : OMAvatarFaceRequest
            The incoming avatar request message.
        """
        if not hasattr(self.orchestrator, "cloud_connection_manager"):
            self.orchestrator.get_logger().warning(
                "Cloud connection manager not available"
            )
            return

        # Broadcast avatar face to WebSocket clients
        avatar_face_data = {
            "type": "avatar",
            "face": msg.face_text,
            "timestamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
        }

        if self.orchestrator.cloud_connection_manager.local_api_ws:
            try:
                self.orchestrator.cloud_connection_manager.local_api_ws.broadcast(
                    json.dumps(avatar_face_data)
                )
                self.orchestrator.get_logger().info(
                    f"Broadcasted avatar face to frontend: {msg.face_text}"
                )
            except Exception as e:
                self.orchestrator.get_logger().error(
                    f"Failed to broadcast avatar face: {e}"
                )

    def avatar_response_callback(self, msg):
        """
        Callback for avatar response messages.

        Parameters:
        ----------
        msg : OMAvatarFaceReponse
            The received avatar response message.
        """
        if not hasattr(self.orchestrator, "api_response_pub"):
            self.orchestrator.get_logger().warning(
                "API response publisher not available"
            )
            return

        response_msg = OMAPIResponse()
        response_msg.header.stamp = self.orchestrator.get_clock().now().to_msg()
        response_msg.request_id = msg.request_id
        response_msg.code = 0
        response_msg.status = "active"
        response_msg.message = msg.message

        self.orchestrator.api_response_pub.publish(response_msg)

    def cloud_api_response_callback(self, response):
        """
        Callback function for OMAPIResponse messages to send to cloud.

        Parameters:
        -----------
        response : om_api.msg.OMAPIResponse
            The incoming OMAPIResponse message containing the API response details.
        """
        if not hasattr(self.orchestrator, "cloud_connection_manager"):
            self.orchestrator.get_logger().warning(
                "Cloud connection manager not available"
            )
            return

        response_data = {
            "time": response.header.stamp.sec + response.header.stamp.nanosec * 1e-9,
            "request_id": response.request_id,
            "code": response.code,
            "status": response.status,
            "message": response.message,
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
        if not hasattr(self.orchestrator, "api_request_pub"):
            self.orchestrator.get_logger().warning(
                "API request publisher not available"
            )
            return

        try:
            msg_json = json.loads(message)

            api_msg = OMAPIRequest()
            api_msg.header.stamp.sec = int(msg_json.get("time", 0))
            api_msg.header.stamp.nanosec = int(
                (msg_json.get("time", 0) - api_msg.header.stamp.sec) * 1e9
            )
            api_msg.request_id = msg_json.get("request_id", str(uuid4()))
            api_msg.action = msg_json.get("action", "")

            parameters = msg_json.get("parameters", "")
            if isinstance(parameters, dict):
                api_msg.parameters = json.dumps(parameters)
            else:
                api_msg.parameters = str(parameters)

            if api_msg.action == "":
                self.orchestrator.get_logger().warning(
                    "Received cloud API message with missing action field."
                )
                return

            self.orchestrator.api_request_pub.publish(api_msg)
            self.orchestrator.get_logger().info(
                f"Published cloud API request: {api_msg.action} with ID {api_msg.request_id}"
            )

        except Exception as e:
            self.orchestrator.get_logger().error(
                f"Failed to process cloud API message: {e}"
            )
