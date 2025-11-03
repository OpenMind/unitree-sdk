from setuptools import find_packages, setup
import os

package_name = 'g1_sdk'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/g1_sdk']),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/g1_control_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='openmind',
    maintainer_email='hello@openmind.org',
    description='Unitree G1 Humanoid Robot ROS2 SDK',
    license='MIT',
    entry_points={
        'console_scripts': [
            'cmd_vel_to_g1 = g1_sdk.g1_teleop:main',
            'g1_loco_action = g1_sdk.g1_loco_action:main',
        ],
    },
)
