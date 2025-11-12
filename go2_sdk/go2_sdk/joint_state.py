import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from unitree_go.msg import LowState


class JointStatePublisher(Node):
    def __init__(self):
        super().__init__("joint_state_publisher")
        self.subscription = self.create_subscription(
            LowState, "/lowstate", self.listener_callback, 10
        )
        self.publisher = self.create_publisher(JointState, "/joint_states", 10)

    def listener_callback(self, msg: LowState):
        """
        Callback function to process incoming LowState messages and publish JointState messages.
        This function extracts joint positions from the LowState message and publishes them
        as a JointState message.

        Parameters:
        -----------
        msg : LowState
            The incoming LowState message containing motor states.
        """
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = [
            "FL_hip_joint",
            "FL_thigh_joint",
            "FL_calf_joint",
            "FR_hip_joint",
            "FR_thigh_joint",
            "FR_calf_joint",
            "RL_hip_joint",
            "RL_thigh_joint",
            "RL_calf_joint",
            "RR_hip_joint",
            "RR_thigh_joint",
            "RR_calf_joint",
        ]
        joint_state.position = [
            msg.motor_state[3].q,
            msg.motor_state[4].q,
            msg.motor_state[5].q,
            msg.motor_state[0].q,
            msg.motor_state[1].q,
            msg.motor_state[2].q,
            msg.motor_state[9].q,
            msg.motor_state[10].q,
            msg.motor_state[11].q,
            msg.motor_state[6].q,
            msg.motor_state[7].q,
            msg.motor_state[8].q,
        ]
        self.publisher.publish(joint_state)


def main(args=None):
    rclpy.init(args=args)

    try:
        node = JointStatePublisher()
        rclpy.spin(node)

    except Exception as e:
        print(f"JointStatePublisher encountered an error: {e}")


if __name__ == "__main__":
    main()
