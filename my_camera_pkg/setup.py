from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_camera_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Add this line so launch files get installed
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='147775420+openminddev@users.noreply.github.com',
    description='USB Camera Launch Package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'apriltag_detector = my_camera_pkg.apriltag_detector:main',
        'go2_apriltag_detector = my_camera_pkg.go2_apriltag_detector:main',
        'go2_tag_follower = my_camera_pkg.go2_tag_follower:main',
        'go2_tag_charger = my_camera_pkg.go2_tag_charger:main',
        'go2_dock = my_camera_pkg.go2_nav_to_charger_final:main',
        'go2_camera_publisher = my_camera_pkg.go2_camera_with_adjustable_publisher:main',
        ],
    },
)
