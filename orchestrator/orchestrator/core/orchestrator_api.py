import os

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from om_api.msg import (
    MapStorage,
    OMAIReponse,
    OMAIRequest,
    OMAPIRequest,
    OMAPIResponse,
    OMAvatarFaceRequest,
    OMAvatarFaceResponse,
    OMModeReponse,
    OMModeRequest,
    OMTTSReponse,
    OMTTSRequest,
    OMConfigRequest,
    OMConfigResponse,
)
from unitree_go.msg import LowState

from ..handlers.api_handlers import APIHandlers
from ..handlers.ros_handlers import ROSHandlers
from ..managers.charging_manager import ChargingManager
from ..managers.location_manager import LocationManager
from ..managers.map_manager import MapManager
from ..managers.process_manager import ProcessManager
from ..models.data_models import ProcessStatus
from ..services.flask_service import FlaskService
from ..services.transform_service import TransformService


class OrchestratorAPI(Node):
    """
    OrchestratorAPI is a ROS2 node that serves as an interface for orchestrating
    various components of the robot system.
    """

    def __init__(self):
        super().__init__("orchestrator_api")

        self.maps_directory = os.path.abspath("./maps")
        self.locations_directory = os.path.abspath("./locations")

        self.base_control_manager = ProcessManager()
        self.slam_manager = ProcessManager()
        self.nav2_manager = ProcessManager()

        self.map_manager = MapManager(self.maps_directory, self.get_logger())
        self.location_manager = LocationManager(
            self.maps_directory, self.locations_directory, self.get_logger()
        )
        self.charging_manager = ChargingManager(self.get_logger())

        self.transform_service = TransformService(self, self.get_logger())
        self.flask_service = FlaskService(logger=self.get_logger())

        self.api_handlers = APIHandlers(self)
        self.ros_handlers = ROSHandlers(self)

        self.flask_service.register_routes(self.api_handlers)

        self._setup_ros_publishers()
        self._setup_ros_subscribers()

        self.flask_service.start()

        self.get_logger().info("Orchestrator API Node has been started.")

    def _setup_ros_publishers(self):
        """
        Setup ROS publishers.
        """
        self.map_saver_pub = self.create_publisher(MapStorage, "/om/map_storage", 10)
        self.api_response_pub = self.create_publisher(
            OMAPIResponse, "/om/api/response", 10
        )
        self.ai_request_pub = self.create_publisher(OMAIRequest, "/om/ai/request", 10)
        self.mode_request_pub = self.create_publisher(
            OMModeRequest, "/om/mode/request", 10
        )
        self.avatar_request_pub = self.create_publisher(
            OMAvatarFaceRequest, "/om/avatar/request", 10
        )
        self.tts_request_pub = self.create_publisher(
            OMTTSRequest, "/om/tts/request", 10
        )
        self.config_request_pub = self.create_publisher(
            OMConfigRequest, "/om/config/request", 10
        )
        self.move_cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

    def _setup_ros_subscribers(self):
        """
        Setup ROS subscribers.
        """
        self.lowstate_sub = self.create_subscription(
            LowState, "/lf/lowstate", self.ros_handlers.lowstate_callback, 10
        )
        self.api_request_sub = self.create_subscription(
            OMAPIRequest, "/om/api/request", self.ros_handlers.api_request_callback, 10
        )
        self.ai_request_sub = self.create_subscription(
            OMAIReponse, "/om/ai/response", self.ros_handlers.ai_response_callback, 10
        )
        self.mode_request_sub = self.create_subscription(
            OMModeReponse,
            "/om/mode/response",
            self.ros_handlers.mode_response_callback,
            10,
        )
        self.avatar_response_sub = self.create_subscription(
            OMAvatarFaceResponse,
            "/om/avatar/response",
            self.ros_handlers.avatar_response_callback,
            10,
        )
        self.tts_request_sub = self.create_subscription(
            OMTTSReponse,
            "/om/tts/response",
            self.ros_handlers.tts_response_callback,
            10,
        )
        self.config_response_sub = self.create_subscription(
            OMConfigResponse,
            "/om/config/response",
            self.ros_handlers.config_response_callback,
            10,
        )

    def is_slam_running(self) -> bool:
        """
        Check if SLAM is currently running.
        """
        return bool(
            self.slam_manager.process and self.slam_manager.process.poll() is None
        )

    def is_nav2_running(self) -> bool:
        """
        Check if Nav2 is currently running.
        """
        return bool(
            self.nav2_manager.process and self.nav2_manager.process.poll() is None
        )

    def is_base_control_running(self) -> bool:
        """
        Check if base control is currently running.
        """
        return bool(
            self.base_control_manager.process
            and self.base_control_manager.process.poll() is None
        )

    def should_start_base_control(self) -> bool:
        """
        Check if base control should be started (when neither SLAM nor Nav2 is running).
        """
        return not self.is_slam_running() and not self.is_nav2_running()

    def manage_base_control(self):
        """
        Start base control if conditions are met, stop it otherwise.
        """
        if self.should_start_base_control() and not self.is_base_control_running():
            self.base_control_manager.start("base_control_launch.py")
            self.get_logger().info("Base control started automatically")
        elif not self.should_start_base_control() and self.is_base_control_running():
            self.base_control_manager.stop()
            self.get_logger().info("Base control stopped automatically")

    def get_process_status(self) -> ProcessStatus:
        """
        Get the current status of all processes.

        Returns:
        -----------
        ProcessStatus
            The current status of SLAM, Nav2, base control, and charging dock processes.
        """
        return ProcessStatus(
            slam_status="running" if self.is_slam_running() else "stopped",
            nav2_status="running" if self.is_nav2_running() else "stopped",
            base_control_status=(
                "running" if self.is_base_control_running() else "stopped"
            ),
            charging_dock_status=(
                "running"
                if self.charging_manager.is_dock_process_running()
                else "stopped"
            ),
            is_charging=self.charging_manager.is_charging,
            battery_soc=self.charging_manager.battery_soc,
            battery_current=self.charging_manager.battery_current,
        )

    def publish_map_storage_message(self, result: dict, map_name: str):
        """
        Publish map storage message for cloud synchronization.

        Parameters:
        -----------
        result : dict
            Result from map saving operation.
        map_name : str
            Name of the saved map.
        """
        files_created = result.get("files_created", [])
        base_path = result.get("base_path", "")

        map_storage_msg = MapStorage()
        map_storage_msg.header.stamp = self.get_clock().now().to_msg()
        map_storage_msg.header.frame_id = "om_api"
        map_storage_msg.map_name = map_name
        map_storage_msg.files_created = files_created
        map_storage_msg.base_path = base_path

        self.map_saver_pub.publish(map_storage_msg)


def main(args=None):
    """
    Main function to initialize the ROS2 api node and spin it.
    """
    rclpy.init(args=args)

    orchestrator_api = OrchestratorAPI()

    try:
        orchestrator_api.manage_base_control()
        rclpy.spin(orchestrator_api)
    except KeyboardInterrupt:
        pass
    finally:
        orchestrator_api.base_control_manager.stop()
        orchestrator_api.slam_manager.stop()
        orchestrator_api.nav2_manager.stop()
        orchestrator_api.charging_manager.stop_dock_sequence()
        orchestrator_api.flask_service.stop()
        orchestrator_api.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
