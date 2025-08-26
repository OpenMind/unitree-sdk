import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_ros import TransformException
from geometry_msgs.msg import PoseStamped

class WaypointManager(Node):
    def __init__(self):
        super().__init__('waypoint_manager')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pose_publisher = self.create_publisher(PoseStamped, '/om/pose', 10)

        self.log_frequency = 1.0
        self.map_frame = 'map'
        self.base_frame = 'base_link'

        self.timer = self.create_timer(1.0 / self.log_frequency, self.publish_pose)

        self.get_logger().info(f'Robot location logger started. Logging at {self.log_frequency} Hz')
        self.get_logger().info(f'Transform: {self.map_frame} -> {self.base_frame}')

    def publish_pose(self):
        """
        Get current robot location and publish it as a PoseStamped message.
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )

            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z

            qx = transform.transform.rotation.x
            qy = transform.transform.rotation.y
            qz = transform.transform.rotation.z
            qw = transform.transform.rotation.w

            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = self.map_frame

            pose_msg.pose.position.x = x
            pose_msg.pose.position.y = y
            pose_msg.pose.position.z = z

            pose_msg.pose.orientation.x = qx
            pose_msg.pose.orientation.y = qy
            pose_msg.pose.orientation.z = qz
            pose_msg.pose.orientation.w = qw

            self.pose_publisher.publish(pose_msg)

            self.get_logger().debug(
                f'Robot Location: X={x:.3f}, Y={y:.3f}, Z={z:.3f} | '
                f'Orientation (Quaternion): [{qx:.3f}, {qy:.3f}, {qz:.3f}, {qw:.3f}]'
            )

        except TransformException as ex:
            self.get_logger().warn(f'Could not get transform: {ex}')
        except Exception as ex:
            self.get_logger().error(f'Error logging robot location: {ex}')

def main(args=None):
    rclpy.init(args=args)

    waypoint_manager = WaypointManager()

    try:
        rclpy.spin(waypoint_manager)
    finally:
        waypoint_manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
