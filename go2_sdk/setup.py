import os
from glob import glob

from setuptools import find_packages, setup

package_name = "go2_sdk"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/go2_sdk"]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*")),
        (
            "share/" + package_name + "/config",
            ["config/slam.yaml", "config/rviz.rviz", "config/nav2_params.yaml"],
        ),
        ("share/" + package_name + "/urdf", ["urdf/go2.urdf"]),
        (os.path.join("share", package_name, "dae"), glob(os.path.join("dae", "*"))),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="openmind",
    maintainer_email="hello@openmind.org",
    description="Unitree Go2 robot SLAM package with RPLidar",
    license="MIT",
    entry_points={
        "console_scripts": [
            "pose_to_tf = go2_sdk.pose_to_tf:main",
            "cmd_vel_to_go2 = go2_sdk.go2_movement:main",
            "waypoint_manager = go2_sdk.waypoint_manager:main",
            "go2_nav2_api = go2_sdk.go2_nav2_api:main",
            "joint_state_publisher = go2_sdk.joint_state:main",
            "d435_obstacle_dector = go2_sdk.d435_obstacle_dector:main",
            "om_path = go2_sdk.om_path:main",
            "go2_sport_action = go2_sdk.go2_sport_action:main",
            "crsf_controller = go2_sdk.crsf_controller:main",
            "go2_camera_stream = go2_sdk.go2_camera_stream:main",
            "d435_camera_stream = go2_sdk.d435_camera_stream:main",
            "go2_lidar_localization = go2_sdk.go2_lidar_localization:main",
        ],
    },
)
