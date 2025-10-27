from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'unitree_go2_auto_dock'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Config files - add this line
        (os.path.join('share', package_name, 'config', 'unitree_go2_cam_indiana'), 
            glob('config/unitree_go2_cam_indiana/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='147775420+openminddev@users.noreply.github.com',
    description='Unitree Go2 Autonomous Docking Package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'apriltag_detector = unitree_go2_auto_dock.apriltag_detector:main',
            'go2_apriltag_detector = unitree_go2_auto_dock.go2_apriltag_detector:main',
            'go2_tag_follower = unitree_go2_auto_dock.go2_tag_follower:main',
            'go2_tag_charger = unitree_go2_auto_dock.go2_tag_charger:main',
            'go2_dock = unitree_go2_auto_dock.go2_nav_to_charger_final:main',
            'go2_camera_publisher = unitree_go2_auto_dock.go2_camera_with_adjustable_publisher:main',
        ],
    },
)