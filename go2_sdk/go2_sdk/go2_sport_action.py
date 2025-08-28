import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from unitree_api.msg import Request, Response, RequestHeader, RequestIdentity

class Go2SportAction(Node):
    """
    A ROS2 node that interfaces with the Go2 sport action system.
    """
    def __init__(self):
        super().__init__("go2_sport_action_node")

        self.SPORT_API_ID_BALANCESTAND = 1002
        self.SPORT_API_ID_STANDUP = 1004
        self.SPORT_API_ID_STANDDOWN = 1005

        self.joy_subscriber = self.create_subscription(
            Joy,
            "/joy",
            self.joy_callback,
            10
        )

        self.sport_publisher = self.create_publisher(
            Request,
            "/api/sport/request",
            10
        )

        self.get_logger().info("Go2 Sport Action Node initialized")

    def joy_callback(self, msg: Joy):
        """
        Callback function for joystick messages.
        Maps joystick buttons to Go2 sport actions.

        Parameters:
        -----------
        msg : sensor_msgs.msg.Joy
            The incoming joystick message containing button states.
        """
        if len(msg.buttons) < 3:
            self.get_logger().warning("Received Joy message with insufficient buttons")
            return

        if msg.buttons[0] == 1:
            self.send_sport_command(self.SPORT_API_ID_STANDUP)
            self.get_logger().info("Sent Stand Up command")
            self.send_sport_command(self.SPORT_API_ID_BALANCESTAND)
            self.get_logger().info("Sent Balance Stand command")

        elif msg.buttons[1] == 1:
            self.send_sport_command(self.SPORT_API_ID_STANDDOWN)
            self.get_logger().info("Sent Stand Down command")

    def send_sport_command(self, api_id: int):
        """
        Sends a command to the Go2 sport action system.

        Parameters:
        -----------
        api_id : int
            The API ID of the command to send.
        """
        request_msg = Request()
        request_msg.header = RequestHeader()
        request_msg.header.identity = RequestIdentity()
        request_msg.header.identity.api_id = api_id

        request_msg.parameter = ""
        self.sport_publisher.publish(request_msg)
        self.get_logger().debug(f"Sent sport command with API ID: {api_id}")

def main(args=None):
    rclpy.init(args=args)

    try:
        node = Go2SportAction()
        rclpy.spin(node)
    except Exception as e:
        print(f"Go2SportAction encountered an error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
