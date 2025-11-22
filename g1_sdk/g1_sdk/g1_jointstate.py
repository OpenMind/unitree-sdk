#!/usr/bin/env python3
import traceback

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from unitree_hg.msg import LowState


class G1JointStatePublisher(Node):
    def __init__(self):
        super().__init__("g1_joint_state_publisher")

        # Publisher for joint states
        self.joint_state_pub = self.create_publisher(JointState, "joint_states", 10)

        # Subscribe to Unitree's low-level state
        self.low_state_sub = self.create_subscription(
            LowState,
            "lowstate",
            self.low_state_callback,
            10,
        )

        # Mapping from motor_state indices to joint names
        # Based on your data showing active motors at indices:
        # 0-5: Left leg, 6-11: Right leg, 12: Waist, 15-19: Left arm, 23-27: Right arm
        self.motor_to_joint_map = {
            # Left leg
            0: "left_hip_pitch_joint",
            1: "left_hip_roll_joint",
            2: "left_hip_yaw_joint",
            3: "left_knee_joint",
            4: "left_ankle_pitch_joint",
            5: "left_ankle_roll_joint",
            # Right leg
            6: "right_hip_pitch_joint",
            7: "right_hip_roll_joint",
            8: "right_hip_yaw_joint",
            9: "right_knee_joint",
            10: "right_ankle_pitch_joint",
            11: "right_ankle_roll_joint",
            # Torso
            12: "waist_yaw_joint",
            # Left arm
            15: "left_shoulder_pitch_joint",
            16: "left_shoulder_roll_joint",
            17: "left_shoulder_yaw_joint",
            18: "left_elbow_joint",
            19: "left_wrist_roll_joint",
            # Right arm (indices 22-26 based on actual data)
            22: "right_shoulder_pitch_joint",
            23: "right_shoulder_roll_joint",
            24: "right_shoulder_yaw_joint",
            25: "right_elbow_joint",
            26: "right_wrist_roll_joint",
        }

        self.msg_count = 0
        self.get_logger().info("G1 Joint State Publisher initialized")
        self.get_logger().info("Subscribing to /lowstate (unitree_hg/msg/LowState)")
        self.get_logger().info("Publishing to /joint_states")

    def low_state_callback(self, msg):
        """Convert Unitree low-level state to ROS JointState"""

        self.msg_count += 1

        try:
            joint_state = JointState()
            joint_state.header = Header()
            joint_state.header.stamp = self.get_clock().now().to_msg()
            joint_state.header.frame_id = ""

            # Initialize lists
            joint_state.name = []
            joint_state.position = []
            joint_state.velocity = []
            joint_state.effort = []

            # Track active motors for debugging
            active_motors = []

            # Process each motor based on our mapping
            for motor_idx, joint_name in self.motor_to_joint_map.items():
                if motor_idx < len(msg.motor_state):
                    motor = msg.motor_state[motor_idx]

                    # Add joint data
                    joint_state.name.append(joint_name)

                    # Get position (q field)
                    if hasattr(motor, "q"):
                        position = float(motor.q)
                        joint_state.position.append(position)

                        # Track active motors (non-zero position or mode==1)
                        if motor.mode == 1:
                            active_motors.append((motor_idx, joint_name, position))
                    else:
                        joint_state.position.append(0.0)

                    # Get velocity (dq field)
                    if hasattr(motor, "dq"):
                        joint_state.velocity.append(float(motor.dq))
                    else:
                        joint_state.velocity.append(0.0)

                    # Get effort (tau_est field)
                    if hasattr(motor, "tau_est"):
                        joint_state.effort.append(float(motor.tau_est))
                    else:
                        joint_state.effort.append(0.0)
                else:
                    # Motor index out of range, add zeros
                    joint_state.name.append(joint_name)
                    joint_state.position.append(0.0)
                    joint_state.velocity.append(0.0)
                    joint_state.effort.append(0.0)

            # Publish joint state
            self.joint_state_pub.publish(joint_state)

        except Exception as e:
            self.get_logger().error(f"Error processing low state: {e}")
            self.get_logger().error(traceback.format_exc())


def main(args=None):
    rclpy.init(args=args)
    node = G1JointStatePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
