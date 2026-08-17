import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    package_share = FindPackageShare('standalone_manipulator')

    robot_description = Command([
        'xacro ',
        PathJoinSubstitution([
            package_share,
            'urdf',
            'standalone_manipulator.urdf.xacro'
        ])
    ])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ])
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': True
            }
        ],
        output='screen'
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'standalone_manipulator'
        ],
        output='screen'
    )


    # Joint State Broadcaster
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )

    # Arm Controller
    arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )

    return LaunchDescription([

        gazebo,

        robot_state_publisher,

        spawn_robot,

        # Wait for Gazebo and ros2_control to initialize
        TimerAction(
            period=3.0,
            actions=[
                joint_state_broadcaster
            ]
        ),

        TimerAction(
            period=5.0,
            actions=[
                arm_controller
            ]
        ),
    ])
