import os
import threading
import time
import docker
from docker.errors import DockerException, NotFound, APIError
import rclpy
import zenoh
from zenoh_sesson import open_zenoh_session
from rclpy.node import Node
import cv2
import subprocess

from sensor_msgs.msg import LaserScan
from om_api.msg import Paths

class WatchSensor(Node):
    def __init__(self):
        super().__init__('watch_sensor')
        self.get_logger().info("WatchSensor node started")

        self.last_scan_time = time.time()
        self.last_paths_time = time.time()
        self.last_zenoh_time = time.time()
        self.last_video_rtsp_time = time.time()
        self.last_audio_rtsp_time = time.time()
        self.timeout_duration = float(os.getenv('SENSOR_TIMEOUT_DURATION', '30.0'))
        self.video_rtsp_url = os.getenv('VIDEO_RTSP_URL', 'rtsp://localhost:8554/top_camera')
        self.audio_rtsp_url = os.getenv('AUDIO_RTSP_URL', 'rtsp://localhost:8554/audio')
        self.video_rtsp_decode_format = os.getenv('VIDEO_RTSP_DECODE_FORMAT', 'H264')

        self.sensor_service_name = os.getenv('SENSOR_SERVICE_NAME', 'om1_sensor')
        self.zenoh_service_name = os.getenv('ZENOH_SERVICE_NAME', 'zenoh_bridge')
        self.video_processor_service_name = os.getenv('VIDEO_PROCESSOR_SERVICE_NAME', 'om1_video_processor')
        self.audio_processor_service_name = os.getenv('AUDIO_PROCESSOR_SERVICE_NAME', 'om1_video_processor')

        try:
            self.docker_client = docker.from_env()
            self.get_logger().info("Docker client initialized successfully")
        except DockerException as e:
            self.get_logger().error(f"Failed to initialize Docker client: {str(e)}")
            raise

        self.watchdog_thread = threading.Thread(target=self.watchdog_loop, daemon=True)
        self.watchdog_thread.start()

        self.video_rtsp_monitor_thread = threading.Thread(target=self.rtsp_monitor_loop, daemon=True)
        self.video_rtsp_monitor_thread.start()

        self.audio_rtsp_monitor_thread = threading.Thread(target=self.audio_rtsp_monitor_loop, daemon=True)
        self.audio_rtsp_monitor_thread.start()

        self.paths_subscription = self.create_subscription(
            Paths,
            '/om/paths',
            self.ros2_paths_callback,
            10
        )

        self.scan_subscription = self.create_subscription(
            LaserScan,
            "/scan",
            self.ros2_scan_callback,
            10,
        )

        self.zenoh_session = open_zenoh_session()
        self.zenoh_session.declare_subscriber("om/paths", self.zenoh_paths_callback)

    def ros2_paths_callback(self, msg: Paths):
        """
        Callback for /om/paths topic

        Parameters
        ----------
        msg : Paths
            The received Paths message
        """
        self.last_paths_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.get_logger().debug("Received paths message")

    def ros2_scan_callback(self, msg: LaserScan):
        """
        Callback for /scan topic

        Parameters
        ----------
        msg : LaserScan
            The received LaserScan message
        """
        self.last_scan_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.get_logger().debug("Received scan message")

    def zenoh_paths_callback(self, sample: zenoh.Sample):
        """
        Callback for /om/paths topic via Zenoh

        Parameters
        ----------
        sample : zenoh.Sample
            The received Zenoh sample
        """
        self.last_zenoh_time = time.time()
        self.get_logger().debug("Received paths message via Zenoh")

    def rtsp_monitor_loop(self):
        """
        Monitor RTSP stream and update last_video_rtsp_time when images are received
        """
        cap = None
        while True:
            try:
                if cap is None:
                    cap = cv2.VideoCapture(self.video_rtsp_url)
                    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                    cap.set(cv2.CAP_PROP_FPS, 5)
                    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc(*self.video_rtsp_decode_format))
                    self.get_logger().info(f"Connected to video RTSP stream: {self.video_rtsp_url}")

                ret, frame = cap.read()
                if ret and frame is not None:
                    self.last_video_rtsp_time = time.time()
                    self.get_logger().info("Received video RTSP frame")
                else:
                    self.get_logger().info("Failed to read video RTSP frame")
                    if cap is not None:
                        cap.release()
                        cap = None

                time.sleep(5.0)

            except Exception as e:
                self.get_logger().error(f"Video RTSP monitoring error: {str(e)}")
                if cap is not None:
                    cap.release()
                    cap = None
                time.sleep(5.0)

    def audio_rtsp_monitor_loop(self):
        """
        Monitor audio RTSP stream and update last_audio_rtsp_time when audio is accessible
        """
        while True:
            try:
                result = subprocess.run([
                    'ffprobe',
                    '-v', 'quiet',
                    '-select_streams', 'a:0',
                    '-show_entries', 'stream=codec_type',
                    '-of', 'csv=p=0',
                    self.audio_rtsp_url
                ],
                capture_output=True,
                text=True,
                timeout=10
                )

                if result.returncode == 0 and 'audio' in result.stdout:
                    self.last_audio_rtsp_time = time.time()
                    self.get_logger().info("Audio RTSP stream is accessible")
                else:
                    self.get_logger().info("Audio RTSP stream not accessible or no audio stream found")

                time.sleep(5.0)

            except subprocess.TimeoutExpired:
                self.get_logger().warning("Audio RTSP stream check timed out")
            except Exception as e:
                self.get_logger().error(f"Audio RTSP monitoring error: {str(e)}")
                time.sleep(5.0)

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
                zenoh_timeout = (current_time - self.last_zenoh_time) > self.timeout_duration
                video_rtsp_timeout = (current_time - self.last_video_rtsp_time) > self.timeout_duration
                audio_rtsp_timeout = (current_time - self.last_audio_rtsp_time) > self.timeout_duration

                if scan_timeout or paths_timeout:
                    missing_topics = []
                    if scan_timeout:
                        missing_topics.append("/scan")
                    if paths_timeout:
                        missing_topics.append("/om/paths")

                    self.get_logger().warn(f"Topics {missing_topics} missing for {self.timeout_duration}s. Restarting containers...")

                    self.restart_container(self.sensor_service_name)
                    time.sleep(1.0)

                    self.last_scan_time = time.time()
                    self.last_paths_time = time.time()

                if zenoh_timeout:
                    self.get_logger().warn(f"Zenoh paths missing for {self.timeout_duration}s. Restarting Zenoh container...")
                    self.restart_container(self.zenoh_service_name)
                    self.last_zenoh_time = time.time()

                if video_rtsp_timeout:
                    self.get_logger().warn(f"RTSP stream not responding for {self.timeout_duration}s. Restarting video processor container...")
                    self.restart_container(self.video_processor_service_name)
                    self.last_video_rtsp_time = time.time()

                if audio_rtsp_timeout:
                    self.get_logger().warn(f"Audio RTSP stream not responding for {self.timeout_duration}s. Restarting audio processor container...")
                    self.restart_container(self.audio_processor_service_name)
                    self.last_audio_rtsp_time = time.time()

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
