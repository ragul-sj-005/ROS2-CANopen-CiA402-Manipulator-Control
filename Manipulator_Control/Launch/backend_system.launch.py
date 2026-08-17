#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        # =====================================================
        # CANopen Manager
        # =====================================================

        Node(
            package='manipulator_ik',
            executable='canopen_manager',
            name='canopen_manager',
            output='screen',
        ),

        # =====================================================
        # CANopen Monitor
        # =====================================================

        Node(
            package='manipulator_ik',
            executable='canopen_monitor',
            name='canopen_monitor',
            output='screen',
        ),

        # =====================================================
        # CANopen Angle Decoder
        # =====================================================

        Node(
            package='manipulator_ik',
            executable='canopen_angle_decoder',
            name='canopen_angle_decoder',
            output='screen',
        ),

        # =====================================================
        # Gazebo Feedback Reader
        # =====================================================

        Node(
            package='manipulator_ik',
            executable='gazebo_feedback_reader',
            name='gazebo_feedback_reader',
            output='screen',
        ),

        # =====================================================
        # CANopen Command Node
        # =====================================================

        Node(
            package='manipulator_ik',
            executable='canopen_command_node',
            name='canopen_command_node',
            output='screen',
        ),

        # =====================================================
        # Gazebo Joint Command Node
        # =====================================================

        Node(
            package='manipulator_ik',
            executable='gazebo_joint_command_node',
            name='gazebo_joint_command_node',
            output='screen',
        ),

    ])
