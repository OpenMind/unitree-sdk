import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class PoseToTF(Node):
    def __init__(self):
        super().__init__('pose_to_tf')

        self.subscription = self.create_subscription(
            PoseStamped,
            '/utlidar/robot_pose',
            self.pose_callback,
            10)

        self.tf_broadcaster = TransformBroadcaster(self)

    def pose_callback(self, msg):
        """
        Callback function for PoseStamped messages.
        Converts PoseStamped message to a TransformStamped message
        and broadcasts it using the tf2_ros TransformBroadcaster.

        Parameters:
        -----------
        msg : geometry_msgs.msg.PoseStamped
            The incoming PoseStamped message containing the robot's pose.
        """
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z
        t.transform.rotation = msg.pose.orientation

        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = PoseToTF()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
