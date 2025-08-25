import rclpy
from rclpy.node import Node
from om_api.msg import MapStorage

class OrchestratorCloud(Node):
    """
    Orchestrator for managing cloud-related tasks.
    """
    def __init__(self):
        super().__init__('orchestrator_cloud')

        self.map_saver_sub = self.create_subscription(
            MapStorage,
            '/om/map_storage',
            self.upload_map,
            10
        )

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

