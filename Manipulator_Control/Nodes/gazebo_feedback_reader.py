#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point


class GazeboFeedbackReader(Node):

    def __init__(self):
        super().__init__('gazebo_feedback_reader')

        self.joint_angles = [0.0, 0.0, 0.0, 0.0]

        # Gazebo joint-state feedback
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # End-effector position feedback
        self.ee_pub = self.create_publisher(
            Point,
            '/gazebo/end_effector_position',
            10
        )

        self.timer = self.create_timer(
            0.1,
            self.publish_feedback
        )

        self.get_logger().info(
            'Gazebo feedback reader started.'
        )

    def joint_state_callback(self, msg):

        joint_names = [
            'joint1',
            'joint2',
            'joint3',
            'joint4'
        ]

        for i, name in enumerate(joint_names):

            if name in msg.name:

                index = msg.name.index(name)

                if index < len(msg.position):
                    self.joint_angles[i] = msg.position[index]

    def forward_kinematics(self):

        q1, q2, q3, q4 = self.joint_angles

        # ------------------------------------------------
        # IMPORTANT:
        # Replace these dimensions/equations with the
        # exact FK model used by your IK node.
        # ------------------------------------------------

        # Example placeholder link dimensions
        L1 = 0.10
        L2 = 0.10
        L3 = 0.10
        L4 = 0.05

        # Example 4-DOF FK
        #
        # q1 = base rotation
        # q2,q3,q4 = arm joints
        #
        # Planar arm angle
        theta2 = q2
        theta3 = q2 + q3
        theta4 = q2 + q3 + q4

        r = (
            L2 * math.cos(theta2)
            + L3 * math.cos(theta3)
            + L4 * math.cos(theta4)
        )

        x = r * math.cos(q1)
        y = r * math.sin(q1)

        z = (
            L1
            + L2 * math.sin(theta2)
            + L3 * math.sin(theta3)
            + L4 * math.sin(theta4)
        )

        return x, y, z

    def publish_feedback(self):

        x, y, z = self.forward_kinematics()

        msg = Point()

        msg.x = x
        msg.y = y
        msg.z = z

        self.ee_pub.publish(msg)

        self.get_logger().info(
            f'Gazebo EE | '
            f'X={x:.4f} m | '
            f'Y={y:.4f} m | '
            f'Z={z:.4f} m'
        )


def main(args=None):

    rclpy.init(args=args)

    node = GazeboFeedbackReader()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
