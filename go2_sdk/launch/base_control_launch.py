from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("go2_sdk")

    return LaunchDescription(
        [
            Node(
                package="go2_sdk",
                executable="cmd_vel_to_go2",
                name="cmd_vel_to_go2",
                output="screen",
            ),
            Node(
                package="go2_sdk",
                executable="go2_sport_action",
                name="go2_sport_action",
                output="screen",
            ),
            Node(
                package="joy", executable="joy_node", name="joy_node", output="screen"
            ),
            # RB is the enable button
            # The left joystick controls linear movement
            # The right joystick controls angular movement
            Node(
                package="teleop_twist_joy",
                executable="teleop_node",
                name="teleop_twist_joy_node",
                output="screen",
                parameters=[
                    {
                        "axis_linear.x": 1,
                        "axis_linear.y": 0,
                        "axis_angular.z": 3,
                        "enable_button": 10,
                        "scale_linear.x": 0.5,
                        "scale_angular.z": 1.0,
                        "enable_turbo_button": 9,
                        "scale_turbo_linear.x": 1.5,
                        "scale_turbo_angular.z": 2.0,
                    }
                ],
            ),
        ]
    )
