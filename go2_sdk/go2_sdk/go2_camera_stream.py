import rclpy
import os
import queue
import threading
from rclpy.node import Node
from unitree_api.msg import Request, RequestHeader, RequestIdentity, Response
import subprocess
import cv2
import numpy as np
import time
from collections import deque

class Go2CameraStreamNode(Node):
    """
    A ROS2 node that interfaces with the Go2 camera system and streams video using FFmpeg.
    """
    def __init__(self):
        super().__init__("go2_camera_stream_node")

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
        self.estimated_fps = 15

        # FPS calculation variables
        self.frame_times = deque(maxlen=30)
        self.last_fps_update = time.time()
        self.current_fps = self.estimated_fps
        self.fps_update_interval = 5.0

        # FFmpeg process management
        self.ffmpeg_process = None
        self.restart_needed = False

        # Queue and threading for frame handling
        self.frame_queue = queue.Queue(maxsize=5)
        self.stop_event = threading.Event()
        self.writer_thread = threading.Thread(target=self._writer_thread, daemon=True)

        # Start FFmpeg and writer thread
        self.setup_ffmpeg_stream()
        self.writer_thread.start()

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

        # 15 Hz timer to request camera images
        self.create_timer(1/15, self.request_camera_image)

        self.get_logger().info("Go2 Camera Stream Node initialized")

    def _calculate_fps(self):
        """
        Calculate actual FPS from recent frame timestamps.
        """
        if len(self.frame_times) < 2:
            return self.estimated_fps

        time_diffs = []
        for i in range(1, len(self.frame_times)):
            time_diffs.append(self.frame_times[i] - self.frame_times[i - 1])

        if time_diffs:
            avg_frame_time = sum(time_diffs) / len(time_diffs)
            if avg_frame_time > 0:
                calculated_fps = 1.0 / avg_frame_time
                return max(5, min(60, calculated_fps))

        return self.current_fps

    def _should_restart_process(self, new_fps: float) -> bool:
        """
        Check if we need to restart FFmpeg due to significant FPS change.
        """
        fps_change = abs(new_fps - self.current_fps) / self.current_fps
        return fps_change > 0.3

    def setup_ffmpeg_stream(self):
        """
        Setup FFmpeg process for video streaming (video only)
        """
        if not self.api_key or not self.api_key_id:
            self.get_logger().error("API key or API key ID not set, cannot start FFmpeg stream")
            return

        if self.ffmpeg_process:
            try:
                if self.ffmpeg_process.stdin and not self.ffmpeg_process.stdin.closed:
                    self.ffmpeg_process.stdin.close()
                self.ffmpeg_process.terminate()
                self.ffmpeg_process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.ffmpeg_process.kill()
                self.ffmpeg_process.wait()
            except Exception:
                pass
            finally:
                self.ffmpeg_process = None

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
            "-g", str(max(15, int(self.current_fps * 2))),
            "-keyint_min", str(max(5, int(self.current_fps / 2))),
            "-vsync", "cfr",

            # --- Output ---
            "-f", "rtsp",
            "-rtsp_transport", "tcp",
            "rtsp://localhost:8554/front_camera"
        ]

        try:
            self.get_logger().info(f"Starting FFmpeg with FPS: {self.current_fps}")
            self.ffmpeg_process = subprocess.Popen(
                ffmpeg_cmd,
                stdin=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                bufsize=self.width * self.height * 3 * 3,
            )
            self.get_logger().info(f"FFmpeg streaming process started with PID: {self.ffmpeg_process.pid}")
            self.restart_needed = False
        except Exception as e:
            self.get_logger().error(f"Failed to start FFmpeg process: {e}")
            self.ffmpeg_process = None

    def request_camera_image(self):
        """
        Sends a request to the Go2 camera system to get an image sample.
        """
        if self.stop_event.is_set():
            return

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
        if self.stop_event.is_set():
            return

        if msg.header.identity.api_id == self.VIDEO_API_ID_GETIMAGESAMPLE:
            image_data = msg.binary
            self.get_logger().debug(f"Received camera image sample of size: {len(image_data)} bytes")

            if self.api_key and self.api_key_id:
                self.process_and_queue_image(image_data)
        else:
            self.get_logger().warning(f"Received unknown response with API ID: {msg.header.identity.api_id}")

    def process_and_queue_image(self, image_data: bytes):
        """
        Process the received image data and queue it for streaming.

        Parameters:
        -----------
        image_data : bytes
            Raw image data from the camera
        """
        try:
            current_time = time.time()
            self.frame_times.append(current_time)

            if current_time - self.last_fps_update > self.fps_update_interval:
                new_fps = self._calculate_fps()

                if self._should_restart_process(new_fps):
                    self.get_logger().info(
                        f"FPS changed significantly: {self.current_fps:.1f} -> {new_fps:.1f}, restarting FFmpeg"
                    )
                    self.current_fps = new_fps
                    self.restart_needed = True
                else:
                    self.get_logger().info(f"Current FPS: {new_fps:.1f}")

                self.last_fps_update = current_time

            nparr = np.frombuffer(image_data, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

            if frame is None:
                self.get_logger().warning("Failed to decode image data")
                return

            if frame.shape[0] != self.height or frame.shape[1] != self.width:
                frame = cv2.resize(frame, (self.width, self.height))

            try:
                while self.frame_queue.qsize() >= 3:
                    try:
                        self.frame_queue.get_nowait()
                    except queue.Empty:
                        break

                self.frame_queue.put_nowait((frame, current_time))

            except queue.Full:
                try:
                    self.frame_queue.get_nowait()
                    self.frame_queue.put_nowait((frame, current_time))
                except queue.Empty:
                    pass

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def _writer_thread(self):
        """
        Thread function to write frames to FFmpeg subprocess.
        """
        while not self.stop_event.is_set():
            try:
                frame_data = self.frame_queue.get(timeout=1)
                frame, _ = frame_data

                if self.restart_needed or not self._is_process_healthy():
                    self.get_logger().warning("Restarting FFmpeg process...")
                    self.setup_ffmpeg_stream()

                    if not self._is_process_healthy():
                        self.get_logger().error("Failed to restart FFmpeg process")
                        continue

                try:
                    self.ffmpeg_process.stdin.write(frame.tobytes())
                    self.ffmpeg_process.stdin.flush()
                except (BrokenPipeError, OSError) as e:
                    self.get_logger().error(f"Pipe error writing frame: {e}")
                    self.restart_needed = True

            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f"Unexpected error in writer thread: {e}")

        # Cleanup when thread stops
        if self.ffmpeg_process:
            try:
                if self.ffmpeg_process.stdin:
                    self.ffmpeg_process.stdin.close()
                self.ffmpeg_process.terminate()
                self.ffmpeg_process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.ffmpeg_process.kill()
                self.ffmpeg_process.wait()
            except Exception:
                pass
            finally:
                self.ffmpeg_process = None

    def _is_process_healthy(self):
        """
        Check if the process is running and available.
        """
        return self.ffmpeg_process is not None and self.ffmpeg_process.poll() is None

    def __del__(self):
        """
        Cleanup on node destruction
        """
        self.stop_event.set()
        if hasattr(self, 'writer_thread'):
            self.writer_thread.join(timeout=5)

        if self.ffmpeg_process:
            try:
                if self.ffmpeg_process.stdin:
                    self.ffmpeg_process.stdin.close()
                self.ffmpeg_process.terminate()
                self.ffmpeg_process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.ffmpeg_process.kill()
                self.ffmpeg_process.wait()
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = Go2CameraStreamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
