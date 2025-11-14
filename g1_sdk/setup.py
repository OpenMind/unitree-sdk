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
        ('share/' + package_name + '/launch', [
            'launch/base_control_launch.py',
            'launch/slam_launch.py',
            'launch/nav2_launch.py'
        ]),
        ('share/' + package_name + '/urdf', [
            'urdf/g1_23dof.urdf',
        ]),
        ('share/' + package_name + '/config', [
            'config/slam.yaml',
            'config/nav2_parameters.yaml',
            
        ]),
        ('share/' + package_name + '/meshes', [
            os.path.join('meshes', f) for f in os.listdir('meshes') if f.endswith('.STL')
        ]),
        # Maps directory for RTAB-Map database storage
        ('share/' + package_name + '/maps', [
            os.path.join('maps', f) for f in os.listdir('maps') if os.path.isfile(os.path.join('maps', f))
        ] if os.path.exists('maps') else []),
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
            'g1_odom = g1_sdk.g1_odom:main',
            'g1_jointstate = g1_sdk.g1_jointstate:main',
            'g1_nav2_api = g1_sdk.g1_nav2_api:main',
            'waypoint_manager = g1_sdk.waypoint_manager:main',
        ],
    },
)