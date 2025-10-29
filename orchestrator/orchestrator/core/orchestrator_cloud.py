import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from om_api.msg import MapStorage, OMAPIRequest, OMAPIResponse

from ..managers.cloud_connection_manager import CloudConnectionManager
from ..managers.map_upload_manager import MapUploadManager
from ..services.cloud_api_service import CloudAPIService
from ..handlers.ros_handlers import ROSHandlers


class OrchestratorCloud(Node):
    """
    OrchestratorCloud is a ROS2 node that manages cloud connectivity,
    including WebSocket connections, map uploads, and cloud API interactions.
    """

    def __init__(self):
        super().__init__('orchestrator_cloud')

        self.cloud_connection_manager = CloudConnectionManager(self.get_logger())
        self.map_upload_manager = MapUploadManager(self.get_logger())
        self.cloud_api_service = CloudAPIService(self.get_logger())

        self.ros_handlers = ROSHandlers(self)

        self._setup_ros_publishers()
        self._setup_ros_subscribers()

        self.cloud_connection_manager.initialize_connections()

        self.cloud_connection_manager.set_api_request_callback(
            self.ros_handlers.cloud_api_request_callback
        )

        self.get_logger().info("Orchestrator Cloud Node Initialized")

    def _setup_ros_publishers(self):
        """
        Setup ROS publishers.
        """
        self.api_request_pub = self.create_publisher(OMAPIRequest, '/om/api/request', 10)

    def _setup_ros_subscribers(self):
        """
        Setup ROS subscribers.
        """
        self.map_saver_sub = self.create_subscription(MapStorage, '/om/map_storage', self.ros_handlers.map_storage_callback, 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/om/pose', self.ros_handlers.cloud_pose_callback, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.ros_handlers.cloud_map_callback, 10)
        self.api_response_sub = self.create_subscription(OMAPIResponse, '/om/api/response', self.ros_handlers.cloud_api_response_callback, 10)

def main(args=None):
    """
    Main function to run the Orchestrator cloud node.
    """
    rclpy.init(args=args)

    orchestrator_cloud = OrchestratorCloud()

    try:
        rclpy.spin(orchestrator_cloud)
    except KeyboardInterrupt:
        pass
    finally:
        orchestrator_cloud.cloud_connection_manager.stop()
        orchestrator_cloud.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

