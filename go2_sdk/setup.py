from setuptools import find_packages, setup

package_name = 'go2_sdk'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/go2_sdk']),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/slam_launch.py', 'launch/nav2_launch.py']),
        ('share/' + package_name + '/config', ['config/slam.yaml', 'config/rviz.rviz', 'config/nav2_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='openmind',
    maintainer_email='hello@openmind.org',
    description='Unitree Go2 robot SLAM package with RPLidar',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_to_tf = go2_sdk.pose_to_tf:main',
            'cmd_vel_to_go2 = go2_sdk.go2_movement:main',
            'waypoint_manager = go2_sdk.waypoint_manager:main',
            'api = go2_sdk.api:main',
        ],
    },
)
