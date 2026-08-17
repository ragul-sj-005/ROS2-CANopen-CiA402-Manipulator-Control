#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class GazeboJointCommandNode(Node):

    def __init__(self):

        super().__init__('gazebo_joint_command_node')

        # =========================================================
        # Joint names
        # =========================================================

        self.joint_names = [
            'joint1',
            'joint2',
            'joint3',
            'joint4'
        ]

        # =========================================================
        # Subscriber
        #
        # Receives:
        #
        # [q1, q2, q3, q4]
        #
        # in radians
        # =========================================================

        self.angle_subscriber = self.create_subscription(
            Float64MultiArray,
            '/decoded_joint_angles',
            self.angle_callback,
            10
        )

        # =========================================================
        # Publisher
        #
        # Change this topic if your Gazebo controller has a
        # different name.
        # =========================================================

        self.trajectory_publisher = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )

        self.get_logger().info(
            '============================================'
        )

        self.get_logger().info(
            '       GAZEBO JOINT COMMAND NODE'
        )

        self.get_logger().info(
            '============================================'
        )

        self.get_logger().info(
            'Subscribed to:'
        )

        self.get_logger().info(
            '  /decoded_joint_angles'
        )

        self.get_logger().info(
            'Publishing to:'
        )

        self.get_logger().info(
            '  /arm_controller/joint_trajectory'
        )

        self.get_logger().info(
            '============================================'
        )

    # =============================================================
    # ANGLE CALLBACK
    # =============================================================

    def angle_callback(self, msg):

        # ---------------------------------------------------------
        # Check number of joint values
        # ---------------------------------------------------------

        if len(msg.data) != 4:

            self.get_logger().error(
                f'Expected 4 joint angles, '
                f'but received {len(msg.data)}'
            )

            return

        q1 = msg.data[0]
        q2 = msg.data[1]
        q3 = msg.data[2]
        q4 = msg.data[3]

        # =========================================================
        # Create JointTrajectory message
        # =========================================================

        trajectory = JointTrajectory()

        # ---------------------------------------------------------
        # Joint names
        # ---------------------------------------------------------

        trajectory.joint_names = self.joint_names

        # =========================================================
        # Create trajectory point
        # =========================================================

        point = JointTrajectoryPoint()

        point.positions = [
            q1,
            q2,
            q3,
            q4
        ]

        # ---------------------------------------------------------
        # Time allowed for movement
        #
        # Example:
        #
        # Current position
        #       ↓
        # Target position
        #       ↓
        # 1 second movement
        # ---------------------------------------------------------

        point.time_from_start.sec = 1
        point.time_from_start.nanosec = 0

        trajectory.points.append(point)

        # =========================================================
        # Publish command
        # =========================================================

        self.trajectory_publisher.publish(trajectory)

        # =========================================================
        # Debug output
        # =========================================================

        self.get_logger().info(
            '--------------------------------------------'
        )

        self.get_logger().info(
            'Gazebo joint command received'
        )

        self.get_logger().info(
            f'joint1 : {q1:.6f} rad'
        )

        self.get_logger().info(
            f'joint2 : {q2:.6f} rad'
        )

        self.get_logger().info(
            f'joint3 : {q3:.6f} rad'
        )

        self.get_logger().info(
            f'joint4 : {q4:.6f} rad'
        )

        self.get_logger().info(
            'JointTrajectory published'
        )

        self.get_logger().info(
            '--------------------------------------------'
        )


def main(args=None):

    rclpy.init(args=args)

    node = GazeboJointCommandNode()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.get_logger().info(
            'Gazebo joint command node stopped.'
        )

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
