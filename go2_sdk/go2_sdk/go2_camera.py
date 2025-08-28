import rclpy
from rclpy.node import Node
from unitree_api.msg import Request, RequestHeader, RequestIdentity, Response

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
            # Process the image data contained in msg.data as needed
        else:
            self.get_logger().warning(f"Received unknown response with API ID: {msg.header.identity.api_id}")


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
