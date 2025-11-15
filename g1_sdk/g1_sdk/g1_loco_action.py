import json

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

from unitree_api.msg import Request, RequestHeader, RequestIdentity


class G1LocoAction(Node):
    """
    A ROS2 node that interfaces with the G1 locomotion action system.
    Provides joystick control for G1 humanoid robot actions.
    """

    def __init__(self):
        super().__init__("g1_loco_action_node")

        # G1 LOCO API IDs (from official SDK)
        self.ROBOT_API_ID_LOCO_SET_FSM_ID = 7101
        self.ROBOT_API_ID_LOCO_SET_BALANCE_MODE = 7102
        self.ROBOT_API_ID_LOCO_SET_ARM_TASK = 7106

        # G1 FSM States (from official SDK)
        self.FSM_ID_ZERO_TORQUE = 0
        self.FSM_ID_DAMP = 1
        self.FSM_ID_SIT = 3
        self.FSM_ID_START = 200  # Official G1 start state
        self.FSM_ID_SQUAT2STANDUP = 706
        self.FSM_ID_STANDUP2SQUAT = 706  # Same as squat2standup
        self.FSM_ID_LIE2STANDUP = 702

        # G1 Balance Modes
        self.BALANCE_MODE_NORMAL = 0
        self.BALANCE_MODE_CONTINUOUS = 1

        # G1 Arm Tasks (from official SDK)
        self.ARM_TASK_WAVE_HAND = 0
        self.ARM_TASK_WAVE_HAND_TURN = 1
        self.ARM_TASK_SHAKE_HAND_1 = 2
        self.ARM_TASK_SHAKE_HAND_2 = 3

        self.joy_subscription = self.create_subscription(
            Joy, "/joy", self.joy_callback, 10
        )

        self.loco_publisher = self.create_publisher(Request, "/api/sport/request", 10)

        self.get_logger().info("G1 Loco Action Node initialized")
        self.get_logger().info("Joystick controls:")
        self.get_logger().info("  Button 0 (A): Squat2StandUp")
        self.get_logger().info("  Button 1 (B): Sit Down")
        self.get_logger().info("  Button 2 (X): Damp Mode")
        self.get_logger().info("  Button 3 (Y): Balance Stand")
        self.get_logger().info("  Button 4 (LB): Wave Hand")
        self.get_logger().info("  Button 5 (RB): Lie2StandUp")

    def joy_callback(self, msg: Joy):
        """
        Callback function for joystick messages.
        Maps joystick buttons to G1 locomotion actions.

        Parameters:
        -----------
        msg : sensor_msgs.msg.Joy
            The incoming joystick message containing button states.
        """
        if len(msg.buttons) < 6:
            self.get_logger().warning("Received Joy message with insufficient buttons")
            return

        # Button 0 (A): Stand Up (Squat to Stand)
        if msg.buttons[0] == 1:
            self.send_fsm_command(self.FSM_ID_SQUAT2STANDUP)
            self.get_logger().info("Sent Stand Up command (Squat2StandUp)")

        # Button 1 (B): Sit Down
        elif msg.buttons[1] == 1:
            self.send_fsm_command(self.FSM_ID_SIT)
            self.get_logger().info("Sent Sit Down command")

        # Button 2 (X): Damp Mode
        elif msg.buttons[2] == 1:
            self.send_fsm_command(self.FSM_ID_DAMP)
            self.get_logger().info("Sent Damp Mode command")

        # Button 3 (Y): Balance Stand
        elif msg.buttons[3] == 1:
            self.send_balance_mode_command(self.BALANCE_MODE_NORMAL)
            self.get_logger().info("Sent Balance Stand command")

        # Button 4 (LB): Wave Hand
        elif msg.buttons[4] == 1:
            self.send_arm_task_command(self.ARM_TASK_WAVE_HAND)
            self.get_logger().info("Sent Wave Hand command")

        # Button 5 (RB): Lie to Stand Up
        elif msg.buttons[5] == 1:
            self.send_fsm_command(self.FSM_ID_LIE2STANDUP)
            self.get_logger().info("Sent Lie to Stand Up command")

    def send_fsm_command(self, fsm_id: int):
        """
        Sends an FSM state command to the G1 locomotion system.

        Parameters:
        -----------
        fsm_id : int
            The FSM state ID to set.
        """
        request_msg = Request()
        request_msg.header = RequestHeader()
        request_msg.header.identity = RequestIdentity()
        request_msg.header.identity.api_id = self.ROBOT_API_ID_LOCO_SET_FSM_ID

        fsm_params = {"data": fsm_id}
        request_msg.parameter = json.dumps(fsm_params)

        self.loco_publisher.publish(request_msg)
        self.get_logger().debug(f"Sent FSM command with ID: {fsm_id}")

    def send_balance_mode_command(self, balance_mode: int):
        """
        Sends a balance mode command to the G1 locomotion system.

        Parameters:
        -----------
        balance_mode : int
            The balance mode to set (0: normal, 1: continuous).
        """
        request_msg = Request()
        request_msg.header = RequestHeader()
        request_msg.header.identity = RequestIdentity()
        request_msg.header.identity.api_id = self.ROBOT_API_ID_LOCO_SET_BALANCE_MODE

        balance_params = {"data": balance_mode}
        request_msg.parameter = json.dumps(balance_params)

        self.loco_publisher.publish(request_msg)
        self.get_logger().debug(f"Sent balance mode command: {balance_mode}")

    def send_arm_task_command(self, task_id: int):
        """
        Sends an arm task command to the G1 locomotion system.

        Parameters:
        -----------
        task_id : int
            The arm task ID to execute.
        """
        request_msg = Request()
        request_msg.header = RequestHeader()
        request_msg.header.identity = RequestIdentity()
        request_msg.header.identity.api_id = self.ROBOT_API_ID_LOCO_SET_ARM_TASK

        arm_params = {"data": task_id}
        request_msg.parameter = json.dumps(arm_params)

        self.loco_publisher.publish(request_msg)
        self.get_logger().debug(f"Sent arm task command: {task_id}")


def main(args=None):
    rclpy.init(args=args)

    try:
        node = G1LocoAction()

        # Initialize G1 to ready state
        node.get_logger().info("Initializing G1 to start state...")
        node.send_fsm_command(node.FSM_ID_START)
        node.get_logger().info(f"Sent FSM_ID_START = {node.FSM_ID_START}")

        rclpy.spin(node)
    except Exception as e:
        print(f"G1LocoAction encountered an error: {e}")
    finally:
        if "node" in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
