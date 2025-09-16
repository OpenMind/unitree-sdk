import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

class OdomRelay(Node):
    def __init__(self):
        super().__init__('odom_relay')

        # Create subscription to odometry
        self.subscription = self.create_subscription(
            Odometry,
            '/utlidar/robot_odom',
            self.odom_callback,
            10)

        # Create publisher for PoseStamped
        self.publisher = self.create_publisher(PoseStamped, '/odom', 10)

    def odom_callback(self, msg):
        """
        Callback function for Odometry messages.
        Converts Odometry message to PoseStamped with system timestamp.

        Parameters:
        -----------
        msg : nav_msgs.msg.Odometry
            The incoming Odometry message containing the robot's pose and twist.
        """
        # Create PoseStamped message with system timestamp
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = msg.header.frame_id  # Use original frame_id or set to 'odom'
        pose_msg.pose = msg.pose.pose  # Extract pose from PoseWithCovariance

        # Publish the PoseStamped message
        self.publisher.publish(pose_msg)

def main():
    rclpy.init()
    node = OdomRelay()  # Fixed class name
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
