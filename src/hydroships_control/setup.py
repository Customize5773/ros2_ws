import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'hydroships_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='HYDROships PPNS',
    maintainer_email='raisyapratama218006@gmail.com',
    description='Kendali & alokasi thruster ROV HYDROships (KKI 2026).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'thruster_allocator = hydroships_control.thruster_allocator:main',
            'teleop_keyboard = hydroships_control.teleop_keyboard:main',
            'stabilizer = hydroships_control.stabilizer:main',
            'teleop_stabilized = hydroships_control.teleop_stabilized:main',
            'depth_publisher = hydroships_control.depth_publisher:main',
            'gripper_controller = hydroships_control.gripper_controller:main',
            'mission_fsm = hydroships_control.mission_fsm:main',
            'qr_detector = hydroships_control.qr_detector:main',
        ],
    },
)
