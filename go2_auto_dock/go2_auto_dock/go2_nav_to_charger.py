#!/usr/bin/env python3

import math

import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


def yaw_from_quaternion(x, y, z, w):
    """Return yaw (degrees) from quaternion."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


def normalize_angle_deg(angle_deg):
    """Normalize angle to [-180, 180] degrees."""
    while angle_deg > 180.0:
        angle_deg -= 360.0
    while angle_deg <= -180.0:
        angle_deg += 360.0
    return angle_deg


class SimpleGoalSender(Node):
    def __init__(self):
        super().__init__("simple_goal_sender")

        # Create action client for NavigateToPose
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # Define goal position and orientation (single source of truth)
        self.goal_position = {"x": -0.7315, "y": -1.1337, "z": 0.0}
        self.goal_orientation = {"x": 0.0, "y": 0.0, "z": -0.7352, "w": 0.6777}
        # Precompute goal yaw (deg) for quick comparison
        self.goal_yaw_deg = yaw_from_quaternion(
            self.goal_orientation["x"],
            self.goal_orientation["y"],
            self.goal_orientation["z"],
            self.goal_orientation["w"],
        )

        self.get_logger().info("Simple Goal Sender initialized")
        self.get_logger().info("Waiting for navigate_to_pose action server...")

    def send_goal(self):
        """Send the navigation goal to the robot"""
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("navigate_to_pose action server not available!")
            return False

        self.get_logger().info("Action server found! Sending goal...")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = "map"

        # Position
        goal_msg.pose.pose.position.x = self.goal_position["x"]
        goal_msg.pose.pose.position.y = self.goal_position["y"]
        goal_msg.pose.pose.position.z = self.goal_position["z"]

        # Orientation
        goal_msg.pose.pose.orientation.x = self.goal_orientation["x"]
        goal_msg.pose.pose.orientation.y = self.goal_orientation["y"]
        goal_msg.pose.pose.orientation.z = self.goal_orientation["z"]
        goal_msg.pose.pose.orientation.w = self.goal_orientation["w"]

        self.get_logger().info("Sending goal to position:")
        self.get_logger().info(f"   X: {goal_msg.pose.pose.position.x:.3f}")
        self.get_logger().info(f"   Y: {goal_msg.pose.pose.position.y:.3f}")
        self.get_logger().info(f"   Z: {goal_msg.pose.pose.position.z:.3f}")

        send_goal_future = self.nav_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
        return True

    def goal_response_callback(self, future):
        """Called when the goal is accepted or rejected"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal was REJECTED by the action server!")
            return

        self.get_logger().info("Goal ACCEPTED by action server!")
        self.get_logger().info("Robot is now navigating to the goal...")
        goal_handle.get_result_async().add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """Called periodically during navigation with progress updates"""
        current_pose = feedback_msg.feedback.current_pose.pose

        # Distance to goal (position only)
        dx = self.goal_position["x"] - current_pose.position.x
        dy = self.goal_position["y"] - current_pose.position.y
        distance_to_goal = math.hypot(dx, dy)

        # Heading error (quaternion vs quaternion -> yaw difference)
        q = current_pose.orientation
        current_yaw_deg = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        heading_error_deg = normalize_angle_deg(self.goal_yaw_deg - current_yaw_deg)

        self.get_logger().info(
            f"Pos: ({current_pose.position.x:.3f}, {current_pose.position.y:.3f}) | "
            f"Dist: {distance_to_goal:.3f} m | "
            f"Yaw: {current_yaw_deg:.1f}° | Goal Yaw: {self.goal_yaw_deg:.1f}° | "
            f"Heading error: {heading_error_deg:.1f}°"
        )

    def get_result_callback(self, future):
        """Called when navigation is complete"""
        result = future.result().result
        if result:
            self.get_logger().info("SUCCESS! Robot has ARRIVED at the goal position!")
            self.get_logger().info("Navigation completed successfully!")
        else:
            self.get_logger().error("FAILED! Robot could not reach the goal position.")
            self.get_logger().error("Navigation was unsuccessful.")


def main():
    rclpy.init()
    node = SimpleGoalSender()
    try:
        if node.send_goal():
            rclpy.spin(node)
        else:
            node.get_logger().error("Failed to send goal. Make sure Nav2 is running!")
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user")
    except Exception as e:
        node.get_logger().error(f"Error occurred: {str(e)}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from rclpy.action import ActionClient
# from nav2_msgs.action import NavigateToPose
# from geometry_msgs.msg import PoseStamped
# import time

# class SimpleGoalSender(Node):
#     def __init__(self):
#         super().__init__('simple_goal_sender')

#         # Create action client for NavigateToPose
#         self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

#         self.get_logger().info('Simple Goal Sender initialized')
#         self.get_logger().info('Waiting for navigate_to_pose action server...')

#     def send_goal(self):
#         """Send the navigation goal to the robot"""

#         # Wait for action server to be available
#         if not self.nav_client.wait_for_server(timeout_sec=10.0):
#             self.get_logger().error('navigate_to_pose action server not available!')
#             return False

#         self.get_logger().info('Action server found! Sending goal...')

#         # Create goal message with your coordinates
#         goal_msg = NavigateToPose.Goal()

#         # Set header
#         goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
#         goal_msg.pose.header.frame_id = 'map'


#         # Set position (your provided coordinates)
#         goal_msg.pose.pose.position.x = 0.24491326860901913
#         goal_msg.pose.pose.position.y = -0.13886247116098704
#         goal_msg.pose.pose.position.z = 0.0

#         # Set orientation (your provided quaternion)
#         goal_msg.pose.pose.orientation.x = 0.0
#         goal_msg.pose.pose.orientation.y = 0.0
#         goal_msg.pose.pose.orientation.z = -0.012617943625043792
#         goal_msg.pose.pose.orientation.w = 0.9999203286680478

#         self.get_logger().info(f'Sending goal to position:')
#         self.get_logger().info(f'   X: {goal_msg.pose.pose.position.x:.3f}')
#         self.get_logger().info(f'   Y: {goal_msg.pose.pose.position.y:.3f}')
#         self.get_logger().info(f'   Z: {goal_msg.pose.pose.position.z:.3f}')

#         # Send goal with callbacks
#         send_goal_future = self.nav_client.send_goal_async(
#             goal_msg,
#             feedback_callback=self.feedback_callback
#         )
#         send_goal_future.add_done_callback(self.goal_response_callback)

#         return True

#     def goal_response_callback(self, future):
#         """Called when the goal is accepted or rejected"""
#         goal_handle = future.result()

#         if not goal_handle.accepted:
#             self.get_logger().error(' Goal was REJECTED by the action server!')
#             return

#         self.get_logger().info('Goal ACCEPTED by action server!')
#         self.get_logger().info('Robot is now navigating to the goal...')

#         # Get the result when navigation is complete
#         get_result_future = goal_handle.get_result_async()
#         get_result_future.add_done_callback(self.get_result_callback)

#     def feedback_callback(self, feedback_msg):
#         """Called periodically during navigation with progress updates"""
#         feedback = feedback_msg.feedback
#         current_pose = feedback.current_pose.pose

#         # Calculate distance to goal
#         goal_x = 0.7051230510266342
#         goal_y = -2.3149364055359545

#         current_x = current_pose.position.x
#         current_y = current_pose.position.y

#         distance_to_goal = ((goal_x - current_x)**2 + (goal_y - current_y)**2)**0.5

#         self.get_logger().info(f' Current position: ({current_x:.3f}, {current_y:.3f}) | Distance to goal: {distance_to_goal:.3f}m')

#     def get_result_callback(self, future):
#         """Called when navigation is complete"""
#         result = future.result().result

#         # Check the result status
#         if result:
#             self.get_logger().info('SUCCESS! Robot has ARRIVED at the goal position! 🎉')
#             self.get_logger().info('Navigation completed successfully!')
#         else:
#             self.get_logger().error('FAILED! Robot could not reach the goal position.')
#             self.get_logger().error('Navigation was unsuccessful.')

# def main():
#     # Initialize ROS2
#     rclpy.init()

#     # Create the goal sender node
#     goal_sender = SimpleGoalSender()

#     try:
#         # Send the goal
#         if goal_sender.send_goal():
#             # Keep spinning to receive callbacks
#             rclpy.spin(goal_sender)
#         else:
#             goal_sender.get_logger().error('Failed to send goal. Make sure Nav2 is running!')

#     except KeyboardInterrupt:
#         goal_sender.get_logger().info(' Interrupted by user')
#     except Exception as e:
#         goal_sender.get_logger().error(f'Error occurred: {str(e)}')
#     finally:
#         # Cleanup
#         goal_sender.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()


# # Set position (your provided coordinates)
# goal_msg.pose.pose.position.x = 0.7051230510266342
# goal_msg.pose.pose.position.y = -2.3149364055359545
# goal_msg.pose.pose.position.z = 0.0001088338940531397

# # Set orientation (your provided quaternion)
# goal_msg.pose.pose.orientation.x = 0.00017121449971607686
# goal_msg.pose.pose.orientation.y = 0.00043085447725235915
# goal_msg.pose.pose.orientation.z = -0.05587689095938943
# goal_msg.pose.pose.orientation.w = 0.9984375584415518
