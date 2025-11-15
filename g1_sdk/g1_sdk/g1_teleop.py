import json
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from unitree_api.msg import Request, RequestHeader, RequestIdentity


class CmdVelToG1LocoNode(Node):
    """
    A ROS2 node that converts cmd_vel (geometry_msgs/Twist) messages
    to Unitree G1 locomotion commands for humanoid robot movement control.
    """

    def __init__(self):
        super().__init__("cmd_vel_to_g1_loco_node")

        # G1 Locomotion API IDs
        self.LOCO_API_ID_SET_FSM_ID = 7101
        self.LOCO_API_ID_SET_BALANCE_MODE = 7102
        self.LOCO_API_ID_SET_STAND_HEIGHT = 7104
        self.LOCO_API_ID_SET_VELOCITY = 7105
        self.LOCO_API_ID_SET_ARM_TASK = 7106

        # FSM States for G1
        self.FSM_STATE_ZEROTORQUE = 0
        self.FSM_STATE_DAMP = 1
        self.FSM_STATE_SIT = 3
        self.FSM_STATE_START = 200
        self.FSM_STATE_LIE2STANDUP = 702
        self.FSM_STATE_STANDUP2SQUAT = 706

        # Create ROS2 interfaces
        self.cmd_vel_subscriber = self.create_subscription(
            Twist, "cmd_vel", self.cmd_vel_callback, 10
        )

        self.loco_publisher = self.create_publisher(
            Request,
            "/api/sport/request",  # Using sport topic as per G1 API service name
            10,
        )

        # Velocity limits for G1 (conservative defaults, adjust based on robot capabilities)
        self.max_linear_x_velocity = 0.5  # m/s forward/backward
        self.max_linear_y_velocity = 0.3  # m/s lateral
        self.max_angular_velocity = 10.0  # rad/s rotation

        # Movement parameters
        self.movement_timeout = 1.0  # seconds
        self.continuous_move = False  # Set to True for continuous movement
        self.last_cmd_time = self.get_clock().now()

        # Safety timer for timeout handling
        self.timeout_timer = self.create_timer(0.1, self.check_timeout_callback)

        # State tracking
        self.is_started = False
        self.is_moving = False

        self.get_logger().info("CmdVel to G1 Locomotion Node initialized")
        self.get_logger().info("Subscribing to: cmd_vel")
        self.get_logger().info("Publishing to: /api/sport/request")
        self.get_logger().info(
            f"Max velocities - X: {self.max_linear_x_velocity} m/s, "
            f"Y: {self.max_linear_y_velocity} m/s, "
            f"Yaw: {self.max_angular_velocity} rad/s"
        )

        # Initialize robot on startup
        self.initialize_robot()

    def initialize_robot(self):
        """
        Initialize the G1 robot to standing position.
        """
        # Send start command to initialize FSM
        self.send_start_command()
        self.get_logger().info("Sent START command to initialize G1 FSM")

    def cmd_vel_callback(self, msg: Twist):
        """
        Callback function for cmd_vel messages.
        Converts Twist message to Unitree G1 locomotion velocity command.

        Parameters:
        -----------
        msg : geometry_msgs.msg.Twist
            The incoming cmd_vel message containing linear and angular velocities.
        """
        self.last_cmd_time = self.get_clock().now()

        # Extract velocities from Twist message
        linear_x = msg.linear.x  # Forward/backward
        linear_y = msg.linear.y  # Left/right (lateral)
        angular_z = msg.angular.z  # Rotation (yaw)

        # Apply velocity limits
        linear_x = max(
            -self.max_linear_x_velocity, min(self.max_linear_x_velocity, linear_x)
        )
        linear_y = max(
            -self.max_linear_y_velocity, min(self.max_linear_y_velocity, linear_y)
        )
        angular_z = max(
            -self.max_angular_velocity, min(self.max_angular_velocity, angular_z)
        )

        # Check if this is effectively a stop command
        velocity_threshold = 0.001
        if (
            abs(linear_x) < velocity_threshold
            and abs(linear_y) < velocity_threshold
            and abs(angular_z) < velocity_threshold
        ):
            self.send_stop_command()
            self.is_moving = False
        else:
            # Send velocity command to G1
            self.send_velocity_command(linear_x, linear_y, angular_z)
            self.is_moving = True

        self.get_logger().debug(
            f"Received cmd_vel - vx: {linear_x:.3f}, vy: {linear_y:.3f}, vyaw: {angular_z:.3f}"
        )

    def send_velocity_command(self, vx: float, vy: float, omega: float):
        """
        Send a velocity command to the G1 robot using the SET_VELOCITY API.

        Parameters:
        -----------
        vx : float
            Forward/backward velocity in m/s
        vy : float
            Lateral velocity in m/s
        omega : float
            Angular velocity in rad/s
        """
        request_msg = Request()
        request_msg.header = RequestHeader()
        request_msg.header.identity = RequestIdentity()
        request_msg.header.identity.api_id = self.LOCO_API_ID_SET_VELOCITY

        # Prepare velocity parameters
        # Duration: 864000.0 for continuous, 1.0 for timed movement
        duration = 864000.0 if self.continuous_move else 1.0

        velocity_params = {
            "velocity": [float(vx), float(vy), float(omega)],
            "duration": duration,
        }

        request_msg.parameter = json.dumps(velocity_params)
        self.loco_publisher.publish(request_msg)

        self.get_logger().debug(
            f"Published velocity command: vx={vx:.3f}, vy={vy:.3f}, "
            f"omega={omega:.3f}, duration={duration}"
        )

    def send_stop_command(self):
        """
        Send a stop movement command to the robot (zero velocities).
        """
        self.send_velocity_command(0.0, 0.0, 0.0)
        self.get_logger().debug("Published stop command (zero velocities)")

    def send_fsm_command(self, fsm_id: int):
        """
        Send an FSM state command to the G1 robot.

        Parameters:
        -----------
        fsm_id : int
            The FSM state ID to transition to
        """
        request_msg = Request()
        request_msg.header = RequestHeader()
        request_msg.header.identity = RequestIdentity()
        request_msg.header.identity.api_id = self.LOCO_API_ID_SET_FSM_ID

        fsm_params = {"data": fsm_id}

        request_msg.parameter = json.dumps(fsm_params)
        self.loco_publisher.publish(request_msg)

        self.get_logger().debug(f"Published FSM command: id={fsm_id}")

    def send_start_command(self):
        """
        Send START command to initialize the G1 robot's FSM.
        """
        self.send_fsm_command(self.FSM_STATE_START)
        self.is_started = True
        self.get_logger().info("G1 FSM started")

    def send_damp_command(self):
        """
        Send DAMP command to put the robot in damping mode.
        """
        self.send_fsm_command(self.FSM_STATE_DAMP)
        self.get_logger().info("G1 set to DAMP mode")

    def send_balance_mode_command(self, balance_mode: int):
        """
        Send a balance mode command to the G1 robot.

        Parameters:
        -----------
        balance_mode : int
            The balance mode to set
        """
        request_msg = Request()
        request_msg.header = RequestHeader()
        request_msg.header.identity = RequestIdentity()
        request_msg.header.identity.api_id = self.LOCO_API_ID_SET_BALANCE_MODE

        balance_params = {"data": balance_mode}

        request_msg.parameter = json.dumps(balance_params)
        self.loco_publisher.publish(request_msg)

        self.get_logger().debug(f"Published balance mode command: mode={balance_mode}")

    def send_stand_height_command(self, height: float):
        """
        Send a stand height command to the G1 robot.

        Parameters:
        -----------
        height : float
            The stand height to set
        """
        request_msg = Request()
        request_msg.header = RequestHeader()
        request_msg.header.identity = RequestIdentity()
        request_msg.header.identity.api_id = self.LOCO_API_ID_SET_STAND_HEIGHT

        height_params = {"data": height}

        request_msg.parameter = json.dumps(height_params)
        self.loco_publisher.publish(request_msg)

        self.get_logger().debug(f"Published stand height command: height={height}")

    def check_timeout_callback(self):
        """
        Timer callback to check for cmd_vel timeout and stop robot if needed.
        """
        current_time = self.get_clock().now()
        time_since_last_cmd = (current_time - self.last_cmd_time).nanoseconds / 1e9

        if self.is_moving and time_since_last_cmd > self.movement_timeout:
            self.send_stop_command()
            self.is_moving = False
            self.get_logger().warn(
                f"Movement timeout ({self.movement_timeout}s) - stopping robot"
            )

    def destroy_node(self):
        """
        Cleanup when node is destroyed.
        """
        # Send stop command before shutting down
        if self.is_moving:
            self.send_stop_command()

        # Optionally put robot in damp mode for safety
        # self.send_damp_command()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    try:
        node = CmdVelToG1LocoNode()

        # Spin the node
        rclpy.spin(node)

    except KeyboardInterrupt:
        print("\nKeyboard interrupt received, shutting down...")
    except Exception as e:
        print(f"CmdVelToG1LocoNode encountered an error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
