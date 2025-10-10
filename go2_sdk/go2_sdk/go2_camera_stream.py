import rclpy
import os
from rclpy.node import Node
from unitree_api.msg import Request, RequestHeader, RequestIdentity, Response
import subprocess
import cv2
import numpy as np
import time
from collections import deque

class Go2CameraNode(Node):
    """
    A ROS2 node that interfaces with the Go2 camera system.
    """
    def __init__(self):
        super().__init__("go2_camera_node")

        self.api_key = os.getenv('OM_API_KEY')
        if not self.api_key:
            self.get_logger().error("OM_API_KEY environment variable not set!")

        self.api_key_id = os.getenv('OM_API_KEY_ID')
        if not self.api_key_id:
            self.get_logger().error("OM_API_KEY_ID environment variable not set!")

        self.VIDEO_API_VERSION = "1.0.0.1"
        self.VIDEO_API_ID_GETIMAGESAMPLE = 1001

        # Camera parameters
        self.width = 680
        self.height = 480
        self.current_fps = 15

        # FPS calculation variables
        self.frame_timestamps = deque(maxlen=30)
        self.last_fps_update = time.time()
        self.fps_update_interval = 5.0

        self.ffmpeg_process = None
        self.setup_ffmpeg_stream()

        self.camera_request_publisher = self.create_publisher(
            Request,
            "/api/videohub/request",
            10
        )

        self.camera_response_subscription = self.create_subscription(
            Response,
            "/api/videohub/response",
            self.camera_response_callback,
            10
        )

        # 30 Hz timer to request camera images
        self.create_timer(1/15, self.request_camera_image)

        self.get_logger().info("Go2 Camera Node initialized")

    def setup_ffmpeg_stream(self):
        """
        Setup FFmpeg process for video streaming (video only)
        """
        if not self.api_key or not self.api_key_id:
            self.get_logger().error("API key or API key ID not set, cannot start FFmpeg stream")
            return

        ffmpeg_cmd = [
            "ffmpeg",
            "-y",
            # --- Video input (raw frames from stdin) ---
            "-f", "rawvideo",
            "-pix_fmt", "bgr24",
            "-s", f"{self.width}x{self.height}",
            "-r", str(self.current_fps),
            "-i", "-",

            # --- Video encoding ---
            "-c:v", "libx264",
            "-preset", "ultrafast",
            "-tune", "zerolatency",
            "-b:v", f"{int(400 * self.current_fps / 15)}k",
            "-g", str(max(15, int(self.current_fps * 2))),  # GOP size
            "-keyint_min", str(max(5, int(self.current_fps / 2))),
            "-vsync", "cfr",

            # --- Output ---
            "-f", "rtsp",
            "-rtsp_transport", "tcp",
            f"rtsp://api-video-ingest.openmind.org:8554/front_camera/{self.api_key_id}"
            f"?api_key={self.api_key}"
        ]

        try:
            self.ffmpeg_process = subprocess.Popen(
                ffmpeg_cmd,
                stdin=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            self.get_logger().info("FFmpeg streaming process started")
        except Exception as e:
            self.get_logger().error(f"Failed to start FFmpeg process: {e}")

    def request_camera_image(self):
        """
        Sends a request to the Go2 camera system to get an image sample.
        """
        request_msg = Request()
        request_msg.header = RequestHeader()
        request_msg.header.identity = RequestIdentity()
        request_msg.header.identity.api_id = self.VIDEO_API_ID_GETIMAGESAMPLE

        request_msg.parameter = ""
        self.camera_request_publisher.publish(request_msg)
        self.get_logger().debug("Requested camera image sample")

    def camera_response_callback(self, msg: Response):
        """
        Callback function to handle responses from the Go2 camera system.

        Parameters:
        -----------
        msg : Response
            The incoming response message containing camera data.
        """
        if msg.header.identity.api_id == self.VIDEO_API_ID_GETIMAGESAMPLE:
            image_data = msg.binary
            self.get_logger().debug(f"Received camera image sample of size: {len(image_data)} bytes")

            if self.api_key and self.api_key_id:
                self.process_and_stream_image(image_data)
        else:
            self.get_logger().warning(f"Received unknown response with API ID: {msg.header.identity.api_id}")

    def process_and_stream_image(self, image_data: bytes):
        """
        Process the received image data and stream it via FFmpeg

        Parameters:
        -----------
        image_data : bytes
            Raw image data from the camera
        """
        try:
            # Record timestamp for FPS calculation
            current_time = time.time()
            self.frame_timestamps.append(current_time)

            # Calculate FPS every second
            if current_time - self.last_fps_update >= self.fps_update_interval:
                self.calculate_fps()
                self.last_fps_update = current_time

            nparr = np.frombuffer(image_data, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

            if frame is not None:
                frame = cv2.resize(frame, (self.width, self.height))

                if self.ffmpeg_process and self.ffmpeg_process.stdin:
                    try:
                        self.ffmpeg_process.stdin.write(frame.tobytes())
                        self.ffmpeg_process.stdin.flush()
                    except BrokenPipeError:
                        self.get_logger().error("FFmpeg process pipe broken, restarting...")
                        self.restart_ffmpeg()
            else:
                self.get_logger().warning("Failed to decode image data")

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def calculate_fps(self):
        """
        Calculate the actual FPS based on frame timestamps
        """
        if len(self.frame_timestamps) >= 2:
            time_span = self.frame_timestamps[-1] - self.frame_timestamps[0]
            if time_span > 0:
                calculated_fps = (len(self.frame_timestamps) - 1) / time_span
                alpha = 0.3  # Smoothing factor
                self.current_fps = alpha * calculated_fps + (1 - alpha) * self.current_fps
                self.get_logger().info(f"Calculated FPS: {self.current_fps:.2f}")

                if abs(calculated_fps - self.current_fps) > 5:
                    self.get_logger().info(f"FPS changed significantly, restarting FFmpeg stream: {self.current_fps:.2f} -> {calculated_fps:.2f}")
                    self.current_fps = calculated_fps
                    self.restart_ffmpeg()

    def restart_ffmpeg(self):
        """
        Restart the FFmpeg process if it fails
        """
        if self.ffmpeg_process:
            try:
                self.ffmpeg_process.terminate()
                self.ffmpeg_process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.ffmpeg_process.kill()
                self.ffmpeg_process.wait()
            except Exception as _:
                pass

        time.sleep(2)

        self.get_logger().info("Restarting FFmpeg process...")
        self.setup_ffmpeg_stream()

    def __del__(self):
        """
        Cleanup FFmpeg process on node destruction
        """
        if self.ffmpeg_process:
            try:
                self.ffmpeg_process.terminate()
                self.ffmpeg_process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.ffmpeg_process.kill()
                self.ffmpeg_process.wait()
            except Exception as _:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = Go2CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
