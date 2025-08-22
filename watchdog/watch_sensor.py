import os
import threading
import time
import docker
from docker.errors import DockerException, NotFound, APIError
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from om_api.msg import Paths

class WatchSensor(Node):
    def __init__(self):
        super().__init__('watch_sensor')
        self.get_logger().info("WatchSensor node started")

        self.last_scan_time = time.time()
        self.last_paths_time = time.time()
        self.timeout_duration = float(os.getenv('SENSOR_TIMEOUT_DURATION', '30.0'))

        self.sensor_service_name = os.getenv('SENSOR_SERVICE_NAME', 'om1_sensor')
        self.zenoh_service_name = os.getenv('ZENOH_SERVICE_NAME', 'zenoh_bridge')

        try:
            self.docker_client = docker.from_env()
            self.get_logger().info("Docker client initialized successfully")
        except DockerException as e:
            self.get_logger().error(f"Failed to initialize Docker client: {str(e)}")
            raise

        self.watchdog_thread = threading.Thread(target=self.watchdog_loop, daemon=True)
        self.watchdog_thread.start()

        self.paths_subscription = self.create_subscription(
            Paths,
            '/om/paths',
            self.paths_callback,
            10
        )

        self.scan_subscription = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            10,
        )

    def paths_callback(self, msg: Paths):
        """
        Callback for /om/paths topic

        Parameters
        ----------
        msg : Paths
            The received Paths message
        """
        self.last_paths_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.get_logger().debug("Received paths message")

    def scan_callback(self, msg: LaserScan):
        """
        Callback for /scan topic

        Parameters
        ----------
        msg : LaserScan
            The received LaserScan message
        """
        self.last_scan_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.get_logger().debug("Received scan message")

    def restart_container(self, service_name: str) -> bool:
        """
        Restart a container

        Parameters
        ----------
        service_name : str
            The name of the service/container to restart

        Returns
        -------
        bool
            True if restart was successful, False otherwise
        """
        try:
            container = self.docker_client.containers.get(service_name)
            self.get_logger().info(f"Restarting container: {container.name}")
            container.restart(timeout=10)
            self.get_logger().info(f"Successfully restarted container: {container.name}")
            return True

        except NotFound:
            self.get_logger().error(f"Container '{service_name}' not found")
            return False
        except APIError as e:
            self.get_logger().error(f"Error restarting {service_name}: {str(e)}")
            return False

    def watchdog_loop(self):
        """
        Monitor topics and restart containers if needed
        """
        while True:
            try:
                current_time = time.time()
                scan_timeout = (current_time - self.last_scan_time) > self.timeout_duration
                paths_timeout = (current_time - self.last_paths_time) > self.timeout_duration

                if scan_timeout or paths_timeout:
                    missing_topics = []
                    if scan_timeout:
                        missing_topics.append("/scan")
                    if paths_timeout:
                        missing_topics.append("/om/paths")

                    self.get_logger().warn(f"Topics {missing_topics} missing for {self.timeout_duration}s. Restarting containers...")

                    self.restart_container(self.sensor_service_name)
                    time.sleep(1.0)
                    self.restart_container(self.zenoh_service_name)

                    self.last_scan_time = time.time()
                    self.last_paths_time = time.time()

                time.sleep(1.0)

            except Exception as e:
                self.get_logger().error(f"Watchdog error: {str(e)}")
                time.sleep(5.0)

    def __del__(self):
        """
        Cleanup Docker client
        """
        try:
            if hasattr(self, 'docker_client'):
                self.docker_client.close()
        except:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = WatchSensor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
