#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
import math
import subprocess
import time
import threading


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
        super().__init__('simple_goal_sender')

        # Create action client for NavigateToPose
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Define goal position and orientation  (bearing angle)
        self.goal_position = {'x': 0.01612, 'y': 0.1392, 'z': 0.0}
        self.goal_orientation = {'x': 0.0, 'y': 0.0, 'z': 0.9512, 'w': 0.3085}

        # Precompute goal yaw (deg)
        self.goal_yaw_deg = yaw_from_quaternion(
            self.goal_orientation['x'],
            self.goal_orientation['y'],
            self.goal_orientation['z'],
            self.goal_orientation['w']
        )

        # Store subprocess references
        self.camera_process = None
        self.detector_process = None
        self.charger_process = None

        # Flag to signal shutdown
        self.should_shutdown = False

        self.get_logger().info('Simple Goal Sender initialized')
        self.get_logger().info('Waiting for navigate_to_pose action server...')

    def send_goal(self):
        """Send the navigation goal to the robot"""
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('navigate_to_pose action server not available!')
            return False

        self.get_logger().info('Action server found! Sending goal...')

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = 'map'

        # Position
        goal_msg.pose.pose.position.x = self.goal_position['x']
        goal_msg.pose.pose.position.y = self.goal_position['y']
        goal_msg.pose.pose.position.z = self.goal_position['z']

        # Orientation
        goal_msg.pose.pose.orientation.x = self.goal_orientation['x']
        goal_msg.pose.pose.orientation.y = self.goal_orientation['y']
        goal_msg.pose.pose.orientation.z = self.goal_orientation['z']
        goal_msg.pose.pose.orientation.w = self.goal_orientation['w']

        self.get_logger().info('Sending goal to position:')
        self.get_logger().info(f'   X: {goal_msg.pose.pose.position.x:.3f}')
        self.get_logger().info(f'   Y: {goal_msg.pose.pose.position.y:.3f}')
        self.get_logger().info(f'   Z: {goal_msg.pose.pose.position.z:.3f}')

        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
        return True

    def goal_response_callback(self, future):
        """Called when the goal is accepted or rejected"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal was REJECTED by the action server!')
            return

        self.get_logger().info('Goal ACCEPTED by action server!')
        self.get_logger().info('Robot is now navigating to the goal...')
        goal_handle.get_result_async().add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """Called periodically during navigation"""
        current_pose = feedback_msg.feedback.current_pose.pose

        dx = self.goal_position['x'] - current_pose.position.x
        dy = self.goal_position['y'] - current_pose.position.y
        distance_to_goal = math.hypot(dx, dy)

        q = current_pose.orientation
        current_yaw_deg = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        heading_error_deg = normalize_angle_deg(self.goal_yaw_deg - current_yaw_deg)

        self.get_logger().info(
            f'Pos: ({current_pose.position.x:.3f}, {current_pose.position.y:.3f}) | '
            f'Dist: {distance_to_goal:.3f} m | '
            f'Yaw: {current_yaw_deg:.1f}° | Goal Yaw: {self.goal_yaw_deg:.1f}° | '
            f'Heading error: {heading_error_deg:.1f}°'
        )

    def monitor_charging_process(self):
        """Monitor the charging process and signal shutdown when complete"""
        if self.charger_process is not None:
            self.get_logger().info('Monitoring charging process...')
            returncode = self.charger_process.wait()  # Block until process completes
            self.get_logger().info(f'Charging process completed with return code: {returncode}')

            # Clean up other processes
            self.cleanup_processes()

            # Signal that we should shutdown
            self.should_shutdown = True

    def cleanup_processes(self):
        """Terminate all spawned processes gracefully"""
        self.get_logger().info('Cleaning up processes...')

        for process, name in [(self.detector_process, 'AprilTag detector'),
                               (self.camera_process, 'Camera')]:
            if process is not None and process.poll() is None:  # Still running
                self.get_logger().info(f'Terminating {name}...')
                process.terminate()
                try:
                    process.wait(timeout=5)
                    self.get_logger().info(f'{name} terminated successfully')
                except subprocess.TimeoutExpired:
                    self.get_logger().warn(f'{name} did not terminate, killing...')
                    process.kill()

    def get_result_callback(self, future):
        """Called when navigation is complete"""
        result = future.result().result
        if result:
            self.get_logger().info('SUCCESS! Robot has ARRIVED at the goal position!')
            self.get_logger().info('Navigation completed successfully!')

            # Start post-navigation tasks
            self.get_logger().info('Starting camera...')
            self.camera_process = subprocess.Popen(
                ['ros2', 'run', 'go2_auto_dock', 'go2_camera_publisher']
            )
            time.sleep(4)

            self.get_logger().info('Starting AprilTag detector...')
            self.detector_process = subprocess.Popen(
                ['ros2', 'run', 'go2_auto_dock', 'go2_apriltag_detector']
            )
            time.sleep(4)

            self.get_logger().info('Starting charging routine...')
            self.charger_process = subprocess.Popen(
                ['ros2', 'run', 'go2_auto_dock', 'go2_tag_charger']
            )

            # Start monitoring thread
            monitor_thread = threading.Thread(target=self.monitor_charging_process, daemon=True)
            monitor_thread.start()

        else:
            self.get_logger().error('FAILED! Robot could not reach the goal position.')
            self.get_logger().error('Navigation was unsuccessful.')
            self.should_shutdown = True


def main():
    rclpy.init()
    node = SimpleGoalSender()
    try:
        if node.send_goal():
            # Spin until shutdown is signaled
            while rclpy.ok() and not node.should_shutdown:
                rclpy.spin_once(node, timeout_sec=0.5)

            node.get_logger().info('Shutting down gracefully...')
        else:
            node.get_logger().error('Failed to send goal. Make sure Nav2 is running!')
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
        node.cleanup_processes()
    except Exception as e:
        node.get_logger().error(f'Error occurred: {str(e)}')
        node.cleanup_processes()
    finally:
        node.destroy_node()
        rclpy.shutdown()
        node.get_logger().info('Shutdown complete!')


if __name__ == '__main__':
    main()
