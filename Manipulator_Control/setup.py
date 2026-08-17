from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'manipulator_ik'

setup(
    name=package_name,
    version='0.0.0',

    packages=find_packages(exclude=['test']),

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),

        (
            'share/' + package_name,
            ['package.xml']
        ),

        # Install all ROS 2 launch files
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')
        ),
    ],

    install_requires=['setuptools'],
    zip_safe=True,

    maintainer='ragulsj005',
    maintainer_email='ragulsj005@todo.todo',

    description='Manipulator IK and CANopen Gazebo integration',
    license='TODO: License declaration',

    extras_require={
        'test': [
            'pytest',
        ],
    },

    entry_points={
        'console_scripts': [
            'ik_node = manipulator_ik.ik_node:main',
            'canopen_manager = manipulator_ik.canopen_manager:main',
            'canopen_command_node = manipulator_ik.canopen_command_node:main',
            'canopen_monitor = manipulator_ik.canopen_monitor:main',
            'canopen_position_reader = manipulator_ik.canopen_position_reader:main',
            'canopen_angle_decoder = manipulator_ik.canopen_angle_decoder:main',
            'gazebo_joint_command_node = manipulator_ik.gazebo_joint_command_node:main',
            'gazebo_feedback_reader = manipulator_ik.gazebo_feedback_reader:main',
        ],
    },
)
