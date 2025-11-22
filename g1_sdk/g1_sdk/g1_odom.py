#!/usr/bin/env python3
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

# Import Unitree ROS2 message
from unitree_go.msg import SportModeState


class UnitreeOdomConverter(Node):
    def __init__(self):
        super().__init__("unitree_odom_converter")

        # Declare parameters
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("use_high_freq", True)
        self.declare_parameter("publish_tf", True)

        # Get parameters
        self.odom_frame_id = self.get_parameter("odom_frame_id").value
        self.base_frame_id = self.get_parameter("base_frame_id").value
        self.use_high_freq = self.get_parameter("use_high_freq").value
        self.publish_tf = self.get_parameter("publish_tf").value

        # Choose topic based on frequency preference
        input_topic = "/odommodestate" if self.use_high_freq else "/lf/odommodestate"

        # Create subscriber to Unitree SportModeState
        self.sport_mode_sub = self.create_subscription(
            SportModeState, input_topic, self.sport_mode_callback, 10
        )

        # Create publishers
        self.odom_publisher = self.create_publisher(Odometry, "/odom", 10)

        # Create TF broadcaster
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info("Unitree Odom Converter initialized")
        self.get_logger().info(f"Subscribing to: {input_topic}")
        self.get_logger().info("Publishing to: /odom")
        self.get_logger().info(
            f"TF: {self.odom_frame_id} -> {self.base_frame_id} (enabled: {self.publish_tf})"
        )

    def sport_mode_callback(self, msg: SportModeState):
        """Convert SportModeState to Odometry and TF"""

        current_time = self.get_clock().now()

        # Create odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = self.odom_frame_id
        odom_msg.child_frame_id = self.base_frame_id

        # Set position from SportModeState
        odom_msg.pose.pose.position.x = float(msg.position[0])
        odom_msg.pose.pose.position.y = float(msg.position[1])
        odom_msg.pose.pose.position.z = float(msg.position[2])

        # Set orientation from IMU quaternion
        # SportModeState quaternion is [w, x, y, z]
        odom_msg.pose.pose.orientation.w = float(msg.imu_state.quaternion[0])
        odom_msg.pose.pose.orientation.x = float(msg.imu_state.quaternion[1])
        odom_msg.pose.pose.orientation.y = float(msg.imu_state.quaternion[2])
        odom_msg.pose.pose.orientation.z = float(msg.imu_state.quaternion[3])

        # Set linear velocity (body frame)
        odom_msg.twist.twist.linear.x = float(msg.velocity[0])
        odom_msg.twist.twist.linear.y = float(msg.velocity[1])
        odom_msg.twist.twist.linear.z = float(msg.velocity[2])

        # Set angular velocity from IMU
        odom_msg.twist.twist.angular.x = float(msg.imu_state.gyroscope[0])
        odom_msg.twist.twist.angular.y = float(msg.imu_state.gyroscope[1])
        odom_msg.twist.twist.angular.z = float(msg.yaw_speed)

        # Set covariance matrices (diagonal matrices with uncertainty values)
        # Adjust these values based on your robot's sensor characteristics

        # Pose covariance (6x6 matrix as 36-element array)
        # Order: (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
        pose_cov = np.zeros(36)
        pose_cov[0] = 0.01  # x
        pose_cov[7] = 0.01  # y
        pose_cov[14] = 0.01  # z
        pose_cov[21] = 0.05  # rot_x
        pose_cov[28] = 0.05  # rot_y
        pose_cov[35] = 0.05  # rot_z
        odom_msg.pose.covariance = pose_cov.tolist()

        # Twist covariance (6x6 matrix as 36-element array)
        # Order: (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
        twist_cov = np.zeros(36)
        twist_cov[0] = 0.01  # vx
        twist_cov[7] = 0.01  # vy
        twist_cov[14] = 0.01  # vz
        twist_cov[21] = 0.05  # wx
        twist_cov[28] = 0.05  # wy
        twist_cov[35] = 0.05  # wz
        odom_msg.twist.covariance = twist_cov.tolist()

        # Publish odometry
        self.odom_publisher.publish(odom_msg)

        # Publish TF transform if enabled
        if self.publish_tf:
            transform = TransformStamped()
            transform.header.stamp = current_time.to_msg()
            transform.header.frame_id = self.odom_frame_id
            transform.child_frame_id = self.base_frame_id

            # Set translation
            transform.transform.translation.x = float(msg.position[0])
            transform.transform.translation.y = float(msg.position[1])
            transform.transform.translation.z = float(msg.position[2])

            # Set rotation
            transform.transform.rotation.w = float(msg.imu_state.quaternion[0])
            transform.transform.rotation.x = float(msg.imu_state.quaternion[1])
            transform.transform.rotation.y = float(msg.imu_state.quaternion[2])
            transform.transform.rotation.z = float(msg.imu_state.quaternion[3])

            # Broadcast transform
            self.tf_broadcaster.sendTransform(transform)

        # Debug logging (uncomment if needed)
        # self.get_logger().info(
        #     f'Odom - Pos: [{odom_msg.pose.pose.position.x:.3f}, '
        #     f'{odom_msg.pose.pose.position.y:.3f}, {odom_msg.pose.pose.position.z:.3f}], '
        #     f'Vel: [{odom_msg.twist.twist.linear.x:.3f}, '
        #     f'{odom_msg.twist.twist.linear.y:.3f}, {odom_msg.twist.twist.linear.z:.3f}]',
        #     throttle_duration_sec=1.0
        # )


def main(args=None):
    rclpy.init(args=args)

    try:
        node = UnitreeOdomConverter()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if "node" in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
