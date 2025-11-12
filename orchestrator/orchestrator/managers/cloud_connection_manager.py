import json
import os
from typing import Callable, Optional
from uuid import uuid4

from ..utils.ws_client import WebSocketClient
from ..utils.ws_server import WebSocketServer

env = os.getenv("ENV", "production")

if env == "development":
    pose_ws_url = "wss://api-dev.openmind.org/api/core/teleops/pose"
    map_ws_url = "wss://api-dev.openmind.org/api/core/teleops/maps"
    api_ws_url = "wss://api-dev.openmind.org/api/core/teleops/api"
else:
    pose_ws_url = "wss://api.openmind.org/api/core/teleops/pose"
    map_ws_url = "wss://api.openmind.org/api/core/teleops/maps"
    api_ws_url = "wss://api.openmind.org/api/core/teleops/api"


class CloudConnectionManager:
    """
    Manages WebSocket connections to cloud services for streaming pose, map, and API data.
    """

    def __init__(self, logger):
        """
        Initialize the cloud connection manager.

        Parameters:
        -----------
        logger : Logger
            Logger instance for logging messages.
        """
        self.logger = logger
        self.api_key = os.getenv("OM_API_KEY")

        if not self.api_key:
            self.logger.error("OM_API_KEY environment variable not set!")

        self.pose_ws_url = (
            f"{pose_ws_url}?api_key={self.api_key}" if self.api_key else None
        )
        self.map_ws_url = (
            f"{map_ws_url}?api_key={self.api_key}" if self.api_key else None
        )
        self.api_ws_url = (
            f"{api_ws_url}?api_key={self.api_key}" if self.api_key else None
        )

        self.pose_ws: Optional[WebSocketClient] = None
        self.map_ws: Optional[WebSocketClient] = None
        self.local_api_ws: Optional[WebSocketServer] = None
        self.cloud_api_ws: Optional[WebSocketClient] = None

        self.api_request_callback: Optional[Callable] = None

    def initialize_connections(self):
        """
        Initialize all WebSocket connections.
        """
        try:
            if self.pose_ws_url:
                self.pose_ws = WebSocketClient(self.pose_ws_url, self.logger)
                self.pose_ws.start()
                self.logger.info("Pose WebSocket connection initialized")

            if self.map_ws_url:
                self.map_ws = WebSocketClient(self.map_ws_url, self.logger)
                self.map_ws.start()
                self.logger.info("Map WebSocket connection initialized")

            self.local_api_ws = WebSocketServer(self.logger, host="0.0.0.0", port=6123)
            self.local_api_ws.start()
            self.logger.info(
                f"Local API WebSocket server started on {self.local_api_ws.host}:{self.local_api_ws.port}"
            )

            if self.api_ws_url:
                self.cloud_api_ws = WebSocketClient(self.api_ws_url, self.logger)
                self.cloud_api_ws.start()
                self.logger.info("Cloud API WebSocket connection initialized")

        except Exception as e:
            self.logger.error(f"Failed to initialize WebSocket connections: {e}")

    def set_api_request_callback(self, callback: Callable):
        """
        Set the callback function for API requests.

        Parameters:
        -----------
        callback : Callable
            Function to call when API requests are received.
        """
        self.api_request_callback = callback

        if self.local_api_ws:
            self.local_api_ws.message_callback = callback

        if self.cloud_api_ws:
            self.cloud_api_ws.message_callback = callback

    def send_pose_data(self, pose_data: dict) -> bool:
        """
        Send pose data to cloud via WebSocket.

        Parameters:
        -----------
        pose_data : dict
            Pose data to send.

        Returns:
        --------
        bool
            True if sent successfully, False otherwise.
        """
        if not self.api_key:
            self.logger.error("Cannot send pose: OM_API_KEY not set.")
            return False

        if self.pose_ws and self.pose_ws.connected:
            try:
                self.pose_ws.send_message(json.dumps(pose_data))
                self.logger.info("Pose data sent to cloud")
                return True
            except Exception as e:
                self.logger.error(f"Failed to send pose data: {e}")
                return False
        else:
            self.logger.warning("Pose WebSocket not connected")
            return False

    def send_map_data(self, map_data: dict) -> bool:
        """
        Send map data to cloud via WebSocket.

        Parameters:
        -----------
        map_data : dict
            Map data to send.

        Returns:
        --------
        bool
            True if sent successfully, False otherwise.
        """
        if not self.api_key:
            self.logger.error("Cannot send map: OM_API_KEY not set.")
            return False

        if self.map_ws and self.map_ws.connected:
            try:
                self.map_ws.send_message(json.dumps(map_data))
                self.logger.info("Map data sent to cloud")
                return True
            except Exception as e:
                self.logger.error(f"Failed to send map data: {e}")
                return False
        else:
            self.logger.warning("Map WebSocket not connected")
            return False

    def send_api_response(self, response_data: dict) -> bool:
        """
        Send API response to cloud and local clients.

        Parameters:
        -----------
        response_data : dict
            Response data to send.

        Returns:
        --------
        bool
            True if sent successfully, False otherwise.
        """
        if not self.api_key:
            self.logger.error("Cannot send API response: OM_API_KEY not set.")
            return False

        success = True

        if self.local_api_ws:
            try:
                self.local_api_ws.broadcast(json.dumps(response_data))
                self.logger.info("API response sent to local clients")
            except Exception as e:
                self.logger.error(f"Failed to send API response to local clients: {e}")
                success = False

        if self.cloud_api_ws and self.cloud_api_ws.connected:
            try:
                self.cloud_api_ws.send_message(json.dumps(response_data))
                self.logger.info("API response sent to cloud")
            except Exception as e:
                self.logger.error(f"Failed to send API response to cloud: {e}")
                success = False
        else:
            self.logger.warning("Cloud API WebSocket not connected")

        return success

    def stop(self):
        """
        Stop all WebSocket connections.
        """
        try:
            if self.pose_ws:
                self.pose_ws.stop()
                self.logger.info("Pose WebSocket stopped")

            if self.map_ws:
                self.map_ws.stop()
                self.logger.info("Map WebSocket stopped")

            if self.local_api_ws:
                self.local_api_ws.stop()
                self.logger.info("Local API WebSocket server stopped")

            if self.cloud_api_ws:
                self.cloud_api_ws.stop()
                self.logger.info("Cloud API WebSocket stopped")

        except Exception as e:
            self.logger.error(f"Error stopping connections: {e}")
