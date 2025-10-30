
import rclpy
from rclpy.node import Node
from unitree_api.msg import Request, RequestHeader, RequestIdentity, Response
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
        self.create_timer(1/30, self.request_camera_image)
        self.get_logger().info("Go2 Camera Node initialized")
    
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
            self.get_logger().info("Received camera image sample")
            
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
            
            self.get_logger().info(f"Binary data length: {len(binary_data)}")
            self.get_logger().debug(f"First 10 values: {binary_data[:10]}")
            
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
            self.get_logger().info(f"Numpy array shape: {nparr.shape}, first few bytes: {nparr[:10]}")
            
            # Check if this looks like a JPEG (starts with 0xFF 0xD8)
            if len(nparr) >= 2 and nparr[0] == 0xFF and nparr[1] == 0xD8:
                self.get_logger().info("Detected JPEG format")
            else:
                self.get_logger().warning(f"Unexpected format, first bytes: {nparr[:10]}")
            
            # Try to decode the image as JPEG
            image = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            
            if image is not None:
                self.get_logger().info(f"Successfully decoded image with shape: {image.shape}")
                # Display the image
                cv2.imshow('Go2 Camera Feed', image)
                
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
        """
        Clean up resources when the node is destroyed.
        """
        cv2.destroyAllWindows()
        super().destroy_node()

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
