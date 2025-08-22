import os
import subprocess
import threading
import time
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
        self.timeout_duration =  float(os.getenv('SENSOR_TIMEOUT_DURATION', '30.0'))

        self.compose_file = os.getenv('COMPOSE_FILE', 'docker-compose.yml')
        self.compose_project_dir = os.getenv('COMPOSE_PROJECT_DIR', '.')
        self.sensor_service_name = os.getenv('SENSOR_SERVICE_NAME', 'om1_sensor')
        self.zenoh_service_name = os.getenv('ZENOH_SERVICE_NAME', 'zenoh_bridge')

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

    def watchdog_loop(self):
        """
        Monitor topics and restart container if needed
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

                    self.get_logger().warn(f"Topics {missing_topics} missing for {self.timeout_duration}s. Restarting container...")
                    self.restart_container()

                    self.last_scan_time = time.time()
                    self.last_paths_time = time.time()

                time.sleep(1.0)

            except Exception as e:
                self.get_logger().error(f"Watchdog error: {str(e)}")
                time.sleep(5.0)

    def restart_container(self):
        """
        Restart the om1_sensor docker container
        """
        try:
            self.get_logger().info(f"Restarting services: {self.sensor_service_name} and {self.zenoh_service_name}")

            original_cwd = os.getcwd()
            os.chdir(self.compose_project_dir)

            subprocess.run([
                'docker-compose', '-f', self.compose_file,
                'rm', '-f', '-s', self.sensor_service_name
            ], check=True, capture_output=True, text=True)

            subprocess.run([
                'docker-compose', '-f', self.compose_file,
                'up', '-d', '--force-recreate', self.sensor_service_name
            ], check=True, capture_output=True, text=True)

            self.get_logger().info(f"Service {self.sensor_service_name} restarted successfully")

            subprocess.run([
                'docker-compose', '-f', self.compose_file,
                'restart', self.zenoh_service_name
            ], check=True, capture_output=True, text=True)

            self.get_logger().info(f"Service {self.zenoh_service_name} restarted successfully")

            os.chdir(original_cwd)

        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Docker-compose error: {e.stderr}")
        except Exception as e:
            self.get_logger().error(f"Failed to restart service: {str(e)}")
        finally:
            try:
                os.chdir(original_cwd)
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
