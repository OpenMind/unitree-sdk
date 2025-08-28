import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from unitree_api.msg import Request, RequestHeader, RequestIdentity
import json

class CmdVelToSportNode(Node):
    """
    A ROS2 node that converts cmd_vel (geometry_msgs/Twist) messages
    to Unitree Go2 sport commands for robot movement control.
    """
    def __init__(self):
        super().__init__("cmd_vel_to_sport_node")

        self.SPORT_API_ID_MOVE = 1008
        self.SPORT_API_ID_BALANCESTAND = 1002
        self.SPORT_API_ID_STOPMOVE = 1003

        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            "cmd_vel",
            self.cmd_vel_callback,
            10
        )

        self.sport_publisher = self.create_publisher(
            Request,
            "/api/sport/request",
            10
        )

        self.max_linear_velocity = 1.0  # m/s
        self.max_angular_velocity = 1.0  # rad/s
        self.movement_timeout = 1.0  # seconds

        self.last_cmd_time = self.get_clock().now()

        self.get_logger().info("CmdVel to Sport Node initialized")
        self.get_logger().info("Subscribing to: cmd_vel")
        self.get_logger().info("Publishing to: /api/sport/request")

    def cmd_vel_callback(self, msg: Twist):
        """
        Callback function for cmd_vel messages.
        Converts Twist message to Unitree sport move command.

        Parameters:
        -----------
        msg : geometry_msgs.msg.Twist
            The incoming cmd_vel message containing linear and angular velocities.
        """
        self.last_cmd_time = self.get_clock().now()

        linear_x = msg.linear.x
        linear_y = msg.linear.y
        angular_z = msg.angular.z

        linear_x = max(-self.max_linear_velocity, min(self.max_linear_velocity, linear_x))
        linear_y = max(-self.max_linear_velocity, min(self.max_linear_velocity, linear_y))
        angular_z = max(-self.max_angular_velocity, min(self.max_angular_velocity, angular_z))

        request_msg = Request()

        request_msg.header = RequestHeader()
        request_msg.header.identity = RequestIdentity()
        request_msg.header.identity.api_id = self.SPORT_API_ID_MOVE

        move_params = {
            "x": float(linear_x),
            "y": float(linear_y),
            "z": float(angular_z)
        }
        request_msg.parameter = json.dumps(move_params)

        self.sport_publisher.publish(request_msg)
        self.get_logger().debug(
            f"Published move command: vx={linear_x:.3f}, vy={linear_y:.3f}, vyaw={angular_z:.3f}"
        )

    def send_stop_command(self):
        """
        Send a stop movement command to the robot.
        """
        request_msg = Request()

        request_msg.header = RequestHeader()
        request_msg.header.identity = RequestIdentity()
        request_msg.header.identity.api_id = self.SPORT_API_ID_STOPMOVE

        request_msg.parameter = ""

        self.sport_publisher.publish(request_msg)
        self.get_logger().debug("Published stop command due to timeout")

    def send_balance_stand_command(self):
        """
        Send a balance stand command to prepare the robot for movement.
        Call this before starting movement commands.
        """
        request_msg = Request()

        request_msg.header = RequestHeader()
        request_msg.header.identity = RequestIdentity()
        request_msg.header.identity.api_id = self.SPORT_API_ID_BALANCESTAND

        request_msg.parameter = ""

        self.sport_publisher.publish(request_msg)
        self.get_logger().info("Published balance stand command")

def main(args=None):
    rclpy.init(args=args)

    try:
        node = CmdVelToSportNode()

        node.send_balance_stand_command()
        rclpy.spin(node)
    except Exception as e:
        print(f"CmdVelToSportNode encountered an error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
