import rclpy
from rclpy.node import Node
from unitree_api.msg import Request, RequestHeader, RequestIdentity, Response
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import base64

class Go2CameraNode(Node):
    """
    A ROS2 node that interfaces with the Go2 camera system.
    """
    def __init__(self):
        super().__init__("go2_camera_node")
        self.VIDEO_API_VERSION = "1.0.0.1"
        self.VIDEO_API_ID_GETIMAGESAMPLE = 1001
        
        # Initialize CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # Configurable output resolution for /camera/image_raw topic
        # Change these values to adjust the published image size
        self.output_width = 640   # Set to 640 for 640x480 or 1920 for 1920x1080
        self.output_height = 480  # Set to 480 for 640x480 or 1080 for 1920x1080
        self.resize_enabled = True  # Set to False to publish original resolution
        
        # Publisher for camera request
        self.camera_request_publisher = self.create_publisher(
            Request,
            "/api/videohub/request",
            10
        )
        
        # Subscriber for camera response
        self.camera_response_subscription = self.create_subscription(
            Response,
            "/api/videohub/response",
            self.camera_response_callback,
            10
        )
        
        # Publisher for processed camera images
        self.image_publisher = self.create_publisher(
            Image,
            "/camera/image_raw",
            10
        )
        
        # 30 Hz timer to request camera images
        self.create_timer(1/30, self.request_camera_image)
        
        if self.resize_enabled:
            self.get_logger().info(f"Go2 Camera Node initialized - Output resolution: {self.output_width}x{self.output_height}")
        else:
            self.get_logger().info("Go2 Camera Node initialized - Publishing original resolution")
    
    def set_output_resolution(self, width, height):
        """
        Dynamically change the output resolution.
        
        Parameters:
        -----------
        width : int
            Target width (e.g., 640, 1920)
        height : int  
            Target height (e.g., 480, 1080)
        """
        self.output_width = width
        self.output_height = height
        self.resize_enabled = True
        self.get_logger().info(f"Output resolution changed to: {width}x{height}")
    
    def disable_resize(self):
        """
        Disable resizing and publish original camera resolution.
        """
        self.resize_enabled = False
        self.get_logger().info("Resize disabled - publishing original resolution")
    
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
            self.get_logger().debug("Received camera image sample")
            
            # Process and display the image data from the binary field
            try:
                self.process_and_display_image(msg.binary)
            except Exception as e:
                self.get_logger().error(f"Error processing image: {str(e)}")
        else:
            self.get_logger().warning(f"Received unknown response with API ID: {msg.header.identity.api_id}")
    
    def process_and_display_image(self, binary_data):
        """
        Process the received binary image data and display it using OpenCV imshow.
        
        Parameters:
        -----------
        binary_data : list
            List of signed integers representing the JPEG image data
        """
        try:
            # Check if data is empty
            if not binary_data:
                self.get_logger().warning("Received empty binary data")
                return
            
            self.get_logger().debug(f"Binary data length: {len(binary_data)}")
            
            # Convert signed integers to unsigned bytes
            # Python signed integers (-128 to 127) need to be converted to unsigned bytes (0 to 255)
            unsigned_bytes = []
            for val in binary_data:
                if val < 0:
                    # Convert negative values: -128 to -1 becomes 128 to 255
                    unsigned_bytes.append(val + 256)
                else:
                    # Positive values stay the same: 0 to 127
                    unsigned_bytes.append(val)
            
            # Convert to bytes
            image_bytes = bytes(unsigned_bytes)
            
            # Convert bytes to numpy array
            nparr = np.frombuffer(image_bytes, np.uint8)
            
            # Try to decode the image as JPEG
            image = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            
            if image is not None:
                original_shape = image.shape
                self.get_logger().debug(f"Decoded image with original shape: {original_shape}")
                
                # Resize image if enabled and different from target resolution
                if self.resize_enabled:
                    if image.shape[1] != self.output_width or image.shape[0] != self.output_height:
                        self.get_logger().debug(f"Resizing from {image.shape[1]}x{image.shape[0]} to {self.output_width}x{self.output_height}")
                        # Use INTER_AREA for downscaling (better quality) or INTER_LINEAR for upscaling
                        interpolation = cv2.INTER_AREA if (self.output_width < image.shape[1]) else cv2.INTER_LINEAR
                        image = cv2.resize(image, (self.output_width, self.output_height), interpolation=interpolation)
                
                # Publish the image to ROS2 topic
                try:
                    # Convert OpenCV image (BGR) to ROS2 Image message
                    ros_image = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
                    
                    # Set timestamp and frame info
                    ros_image.header.stamp = self.get_clock().now().to_msg()
                    ros_image.header.frame_id = "camera_frame"
                    
                    # Publish the image
                    self.image_publisher.publish(ros_image)
                    self.get_logger().debug(f"Published image: {image.shape[1]}x{image.shape[0]} to /camera/image_raw")
                    
                except Exception as e:
                    self.get_logger().error(f"Error publishing ROS image: {str(e)}")
                
                # Optional: Display the image using OpenCV (uncomment if needed)
                # cv2.imshow('Go2 Camera Feed', image)
                
                # Wait for 1ms and check for 'q' key press to quit
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    self.get_logger().info("User requested to quit")
                    rclpy.shutdown()
                    
            else:
                self.get_logger().error("Failed to decode image data - cv2.imdecode returned None")
                
        except Exception as e:
            self.get_logger().error(f"Error in process_and_display_image: {str(e)}")
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def destroy_node(self):
        """`
        Clean up resources when the node is destroyed.
        """
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = Go2CameraNode()
    
    # Example: You can change resolution during runtime
    # Uncomment these lines to test dynamic resolution changes:
    
    # For 640x480 output:
    # node.set_output_resolution(640, 480)
    
    # For 1920x1080 output:
    # node.set_output_resolution(1920, 1080)
    
    # For original resolution (no resizing):
    # node.disable_resize()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()