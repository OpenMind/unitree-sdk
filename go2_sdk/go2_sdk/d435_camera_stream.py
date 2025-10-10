import rclpy
import os
from rclpy.node import Node
import subprocess
import cv2
import numpy as np
import time
from collections import deque
from sensor_msgs.msg import Image

class D435RGBStream(Node):
    """
    A ROS2 node that reads RGB images from the D435 camera and streams video using FFmpeg.
    """
    def __init__(self):
        super().__init__("d435_rgb_stream_node")

        self.api_key = os.getenv('OM_API_KEY')
        if not self.api_key:
            self.get_logger().error("OM_API_KEY environment variable not set!")

        self.api_key_id = os.getenv('OM_API_KEY_ID')
        if not self.api_key_id:
            self.get_logger().error("OM_API_KEY_ID environment variable not set!")

        # Camera parameters
        self.width = 424
        self.height = 240
        self.current_fps = 15

        # FPS calculation variables
        self.frame_timestamps = deque(maxlen=15)
        self.last_fps_update = time.time()
        self.fps_update_interval = 5.0

        self.ffmpeg_process = None
        self.setup_ffmpeg_stream()

        self.camera_response_subscription = self.create_subscription(
            Image,
            "/camera/realsense2_camera_node/color/image_raw",
            self.camera_response_callback,
            10
        )

        self.get_logger().info("D435 RGB stream node initialized")

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
            "-f", "tee",
            "-map", "0:v",
            "[f=rtsp:rtsp_transport=tcp]rtsp://localhost:8554/down_camera"
            "|"
            f"[f=rtsp:rtsp_transport=tcp]rtsp://api-video-ingest.openmind.org:8554/down_camera/{self.api_key_id}?api_key={self.api_key}"
        ]

        try:
            self.ffmpeg_process = subprocess.Popen(
                ffmpeg_cmd,
                stdin=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=self.width * self.height * 3 * 3,
            )
            self.get_logger().info("FFmpeg streaming process started")
        except Exception as e:
            self.get_logger().error(f"Failed to start FFmpeg process: {e}")

    def camera_response_callback(self, msg: Image):
        """
        Callback function to handle responses from the Go2 camera system.

        Parameters:
        -----------
        msg : Image
            The image message received from the camera topic.
        """
        image_data = msg.data
        self.get_logger().debug(f"Received camera image sample of size: {len(image_data)} bytes")

        if self.api_key and self.api_key_id:
            self.process_and_stream_image(image_data)

    def process_and_stream_image(self, image_data: bytes):
        """
        Process the received image data and stream it via FFmpeg

        Parameters:
        -----------
        image_data : bytes
            Raw image data from the camera
        """
        try:
            current_time = time.time()
            self.frame_timestamps.append(current_time)

            if current_time - self.last_fps_update >= self.fps_update_interval:
                self.calculate_fps()
                self.last_fps_update = current_time

            nparr = np.frombuffer(image_data, dtype=np.uint8)
            frame = nparr.reshape((self.height, self.width, 3))

            if frame is None or frame.size == 0:
                self.get_logger().warning("Invalid frame data received")
                return

            if frame.shape[0] != self.height or frame.shape[1] != self.width:
                frame = cv2.resize(frame, (self.width, self.height))

                if not self._is_process_healthy():
                    self.get_logger().warning("FFmpeg process not healthy, restarting...")
                    self.restart_ffmpeg()
                    return

            if self.ffmpeg_process and self.ffmpeg_process.stdin:
                try:
                    self.ffmpeg_process.stdin.write(frame.tobytes())
                    self.ffmpeg_process.stdin.flush()
                except BrokenPipeError:
                    self.get_logger().error("FFmpeg process pipe broken, restarting...")
                    self.restart_ffmpeg()
                except Exception as e:
                    self.get_logger().error(f"Error writing to FFmpeg: {e}")
                    self.restart_ffmpeg()
            else:
                self.get_logger().warning("FFmpeg process not available")

        except ValueError as e:
            self.get_logger().error(f"Error reshaping image data: {e}. Expected size: {self.height * self.width * 3}, got: {len(image_data)}")
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

    def _is_process_healthy(self):
        """
        Check if the process is running and available.
        """
        return self.ffmpeg_process is not None and self.ffmpeg_process.poll() is None

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
    node = D435RGBStream()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
