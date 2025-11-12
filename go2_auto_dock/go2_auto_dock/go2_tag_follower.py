#!/usr/bin/env python3
import json
import math
from time import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from unitree_api.msg import Request, RequestHeader, RequestIdentity


class Go2TagFollower(Node):
    def __init__(self):
        super().__init__("go2_tag_follower")

        # ===== Unitree Sport API IDs (match your reference) =====
        self.ROBOT_SPORT_API_ID_MOVE = 1008
        self.ROBOT_SPORT_API_ID_BALANCESTAND = 1002
        self.ROBOT_SPORT_API_ID_STOPMOVE = 1003
        self.ROBOT_SPORT_API_ID_SIT = 1005  #  StandSit

        # ===== I/O =====
        self.subscription = self.create_subscription(
            Float32MultiArray, "/apriltag_relative", self.tag_callback, 10
        )
        self.sport_pub = self.create_publisher(Request, "/api/sport/request", 10)

        # ===== Control targets =====
        self.forward_target_slow = -0.70
        self.last_move_time = 0
        self.move_delay = 1.8  # Wait 1 second between moves
        self.forward_target_min = -0.43  # want z_forward in [-0.35, -0.30]
        self.forward_target_max = -0.30
        self.forward_target_center = -0.30  # ideal target

        self.right_target_min = -0.045  # want x_right in [-0.05, 0.05]
        self.right_target_max = 0.045

        self.yaw_angle_min = -10.0  # left
        self.yaw_angle_max = 10.0  # right

        # ===== Timing / state =====

        # 1 Hz control/supervisor loop
        self.create_timer(0.8, self.timer_callback)  # 0.5 = 2 hz

        # ~5 degrees per step (used as angular velocity for ~1s → ~5°)
        self.small_turn_rad = math.radians(5)  # ≈ 0.087 rad

        # Optional: put robot into balance-stand at startup
        self.send_balance_stand_command()
        self.get_logger().info(
            "Go2TagFollower initialized (1 Hz, ~5° turns, 30s timeout)"
        )

        # Arrival validation counter
        self.zone_counter = 0  # must reach 3 to confirm arrival

        self.arrived = False
        self.latest_tag = None
        self.last_seen_time = time()  # timestamp of last detection
        # Lost tag threshold (seconds)
        self.lost_threshold = 2.0  # 1 sec tolerance

        # search algorithm
        self.search_turn_side = 0  # decide turn direction
        self.search_turn_deg = 10.0  # start turning 2 degrees
        self.max_search_deg = 31.0  # maximum turning degree
        self.turn_increment = 10.0  # how much to increase each time
        self.backward_after_max = 0.25  # backward step when max reached

        self.continuous_back_count = 0  # when reach 3 times, move large back to escape
        self.increase_back_step_after_continuous_back = (
            0.3  # to prevent getting stuck in the same sport
        )

    # ------------ Unitree API helpers ------------

    def _publish_request(self, api_id: int, params_dict=None):
        """Low-level helper to publish a Unitree Request with JSON 'parameter'."""
        req = Request()
        req.header = RequestHeader()
        req.header.identity = RequestIdentity()
        req.header.identity.api_id = api_id
        req.parameter = json.dumps(params_dict) if params_dict is not None else ""
        self.sport_pub.publish(req)

    def send_move(self, x: float, y: float, yaw_rad_s: float):
        """
        Send a move command.
        Unitree uses parameter keys: x, y, z  (z == yaw rate).
        """
        params = {"x": float(x), "y": float(y), "z": float(yaw_rad_s)}
        self._publish_request(self.ROBOT_SPORT_API_ID_MOVE, params)
        self.get_logger().info(
            f"MOVE: x={params['x']:.2f}, y={params['y']:.2f}, yaw(z)={math.degrees(params['z']):.2f}"
        )

    def send_stop_command(self):
        """Send stop movement command."""
        self._publish_request(self.ROBOT_SPORT_API_ID_STOPMOVE)
        self.get_logger().info("STOP command sent")

    def send_balance_stand_command(self):
        """Prepare robot for movement."""
        self._publish_request(self.ROBOT_SPORT_API_ID_BALANCESTAND)
        self.get_logger().info("BALANCE_STAND command sent")

    def send_sit_command(self):
        """Command robot to sit down."""
        self._publish_request(self.ROBOT_SPORT_API_ID_SIT)
        self.get_logger().info("SIT command sent")

    # ------------ Subscriber callback ------------  30 hz
    def tag_callback(self, msg: Float32MultiArray):
        # Save latest tag and update timestamp
        self.latest_tag = (
            msg.data
        )  # [x_right, y_up, z_forward, yaw_error, self.current_hz, see_target, self.bearing_angle_deg_high_accurate, see_angle]   8 parameters
        see_target = msg.data[5]
        if int(see_target) == 1:
            self.last_seen_time = time()
            self.continuous_back_count = 0

    # ------------ Timer / control loop ------------
    def timer_callback(self):
        now = time()

        # --- Check for tag loss ---
        time_since_seen = now - self.last_seen_time

        if self.arrived:
            return

        if self.latest_tag is None or time_since_seen > self.lost_threshold:
            self.get_logger().info(
                f"Tag lost = performing search maneuver {time_since_seen:.2f} > {self.lost_threshold:.2f}"
            )
            # self.send_move(-0.05, 0.0, self.small_turn_rad if int(now) % 2 == 0 else -self.small_turn_rad)

            # Compute turn (left = -, right = +)
            direction = -1 if self.search_turn_side == 0 else 1
            turn_rad = math.radians(self.search_turn_deg) * direction

            self.send_move(0.0, 0.0, turn_rad)

            # Switch side for next time
            if self.search_turn_side == 0:
                # Just did left → next right (same magnitude)
                self.search_turn_side = 1
            else:
                # Just did right → next left, increase magnitude
                self.search_turn_side = 0
                self.search_turn_deg += self.turn_increment

                # Check if reached max
                if self.search_turn_deg > self.max_search_deg:
                    # Step backward and reset
                    self.continuous_back_count += 1  # reset after see the marker again

                    if (
                        self.continuous_back_count == 3
                    ):  # third time, move bigger step to escape
                        self.send_move(
                            -self.increase_back_step_after_continuous_back, 0.0, 0.0
                        )
                        print("move larger back, because continuous back")
                    else:
                        self.send_move(-self.backward_after_max, 0.0, 0.0)

                    self.search_turn_deg = 10.0  # reset back to 10
                    self.search_turn_side = 0  # state machine
            return

        # --- Extract latest measurements ---
        (
            x_right,
            _,
            z_forward,
            yaw_error,
            _,
            see_target,
            bearing_angle,
            see_angle,
        ) = self.latest_tag

        if int(see_target) == 0:  # eqaupl 0 , no target count
            self.get_logger().info(
                f"Tag lost = {time_since_seen:.2f} > {self.lost_threshold:.2f}"
            )
            return

        self.get_logger().info(
            f"Target: x_right={x_right:.2f}, z_forward={z_forward:.2f}, bearing_degree={bearing_angle:.2f}"
        )

        # --- Forward/backward control ---
        x_cmd = 0.0

        if z_forward < self.forward_target_min:  # < -0.43
            # x_cmd = 0.23
            error = (
                self.forward_target_center - z_forward
            )  # e.g. -0.35 - (-0.50) = 0.15        -0.35 - (-0.60) = 0.25
            x_cmd = error  # directly use error, or clamp to max step
            x_cmd = min(x_cmd, 0.23)  # if last step keep geasture for 2 second    0.25

            if (
                z_forward > self.forward_target_slow
            ):  # slow down when the robot is close to the target, less than  -0.7 ~ -0.43
                if now - self.last_move_time < self.move_delay:
                    print("Slow Down Close to Target")
                    return  # Skip this control cycle
                self.last_move_time = now  # send command, update current time

        elif z_forward > self.forward_target_max:
            x_cmd = -0.2

        # --- Lateral control ---
        y_cmd = 0.0
        if x_right < self.right_target_min:
            y_cmd = -0.2
        elif x_right > self.right_target_max:
            y_cmd = 0.2

        # --- Yaw control ---
        yaw_cmd_radian = 0.0
        if int(see_angle) == 1:  # see target
            if abs(bearing_angle) > 10.0:
                x_cmd = 0.5 * x_cmd
                yaw_cmd_degree = max(
                    -9, min(9, 0.8 * bearing_angle)
                )  # proportional adjustment change 30 degree to radian
                yaw_cmd_radian = math.radians(yaw_cmd_degree) * (
                    -1
                )  # becaues unitree turn left is positive
            # if  bearing_angle < self.yaw_angle_min:
            #     yaw_cmd_radian = np.deg2rad(-30)
            # elif bearing_angle  > self.yaw_angle_max:
            #     yaw_cmd_radian = np.deg2rad(30)

        # --- Check if inside charging zone ---
        in_forward = self.forward_target_min <= z_forward <= self.forward_target_max
        in_right = self.right_target_min <= x_right <= self.right_target_max

        if in_forward and in_right:
            self.zone_counter += 1
            self.get_logger().debug(
                f"In charging zone candidate ({self.zone_counter}/3)"
            )
            if self.zone_counter >= 3:
                self.get_logger().info("Confirmed arrival in charging zone!")
                self.send_stop_command()
                self.send_sit_command()
                self.arrived = True
            return
        else:
            if self.zone_counter > 0:
                self.get_logger().debug("Left zone → reset counter")
            self.zone_counter = 0

        # --- Send movement command ---
        self.send_move(x_cmd, y_cmd, yaw_cmd_radian)


def main(args=None):
    rclpy.init(args=args)
    node = Go2TagFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
