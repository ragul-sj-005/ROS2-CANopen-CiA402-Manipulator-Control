#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point


# ============================================================
# ROTATION MATRICES
# ============================================================

def rot_z(theta):

    c = math.cos(theta)
    s = math.sin(theta)

    return [
        [c, -s, 0.0],
        [s,  c, 0.0],
        [0.0, 0.0, 1.0]
    ]


def rot_y(theta):

    c = math.cos(theta)
    s = math.sin(theta)

    return [
        [ c, 0.0, s],
        [0.0, 1.0, 0.0],
        [-s, 0.0, c]
    ]


# ============================================================
# MATRIX OPERATIONS
# ============================================================

def matmul(A, B):

    return [
        [
            sum(
                A[i][k] * B[k][j]
                for k in range(3)
            )
            for j in range(3)
        ]
        for i in range(3)
    ]


def matvec(R, v):

    return [
        sum(
            R[i][j] * v[j]
            for j in range(3)
        )
        for i in range(3)
    ]


# ============================================================
# FORWARD KINEMATICS
#
# Used ONLY for:
# 1. Numerical IK
# 2. IK solution verification
#
# It is NOT used to calculate Gazebo feedback.
# ============================================================

def forward_kinematics(q1, q2, q3, q4):

    base_height = 0.05

    joint1_offset = [0.012, 0.0, 0.017]
    joint2_offset = [0.0, 0.0, 0.0595]
    joint3_offset = [0.024, 0.0, 0.128]
    joint4_offset = [0.124, 0.0, 0.0]
    ee_offset = [0.126, 0.0, 0.0]

    R1 = rot_z(q1)
    R2 = rot_y(q2)
    R3 = rot_y(q3)
    R4 = rot_y(q4)

    position = [
        0.0,
        0.0,
        base_height
    ]

    # --------------------------------------------------------
    # Joint 1 offset
    # --------------------------------------------------------

    offset = matvec(
        R1,
        joint1_offset
    )

    position = [
        position[i] + offset[i]
        for i in range(3)
    ]

    # --------------------------------------------------------
    # Joint 2 offset
    # --------------------------------------------------------

    R = R1

    offset = matvec(
        R,
        joint2_offset
    )

    position = [
        position[i] + offset[i]
        for i in range(3)
    ]

    # --------------------------------------------------------
    # Joint 3 offset
    # --------------------------------------------------------

    R = matmul(
        R,
        R2
    )

    offset = matvec(
        R,
        joint3_offset
    )

    position = [
        position[i] + offset[i]
        for i in range(3)
    ]

    # --------------------------------------------------------
    # Joint 4 offset
    # --------------------------------------------------------

    R = matmul(
        R,
        R3
    )

    offset = matvec(
        R,
        joint4_offset
    )

    position = [
        position[i] + offset[i]
        for i in range(3)
    ]

    # --------------------------------------------------------
    # End-effector offset
    # --------------------------------------------------------

    R = matmul(
        R,
        R4
    )

    offset = matvec(
        R,
        ee_offset
    )

    position = [
        position[i] + offset[i]
        for i in range(3)
    ]

    return (
        position[0],
        position[1],
        position[2]
    )


# ============================================================
# NUMERICAL JACOBIAN
# ============================================================

def numerical_jacobian(q):

    delta = 1e-6

    x0, y0, z0 = forward_kinematics(*q)

    J = []

    for i in range(4):

        q_plus = q.copy()

        q_plus[i] += delta

        x, y, z = forward_kinematics(*q_plus)

        J.append([
            (x - x0) / delta,
            (y - y0) / delta,
            (z - z0) / delta
        ])

    return [
        [
            J[j][i]
            for j in range(4)
        ]
        for i in range(3)
    ]


# ============================================================
# MATRIX FUNCTIONS
# ============================================================

def transpose(A):

    return [
        [
            A[j][i]
            for j in range(len(A))
        ]
        for i in range(len(A[0]))
    ]


def matmul_general(A, B):

    return [
        [
            sum(
                A[i][k] * B[k][j]
                for k in range(len(B))
            )
            for j in range(len(B[0]))
        ]
        for i in range(len(A))
    ]


def matvec_general(A, v):

    return [
        sum(
            A[i][j] * v[j]
            for j in range(len(v))
        )
        for i in range(len(A))
    ]


def add_damping(A, damping):

    result = [
        row[:]
        for row in A
    ]

    for i in range(len(result)):

        result[i][i] += damping

    return result


def inverse_3x3(A):

    a, b, c = A[0]
    d, e, f = A[1]
    g, h, i = A[2]

    det = (
        a * (e * i - f * h)
        - b * (d * i - f * g)
        + c * (d * h - e * g)
    )

    if abs(det) < 1e-12:

        raise ValueError(
            "Matrix is singular."
        )

    return [

        [
            (e * i - f * h) / det,
            (c * h - b * i) / det,
            (b * f - c * e) / det
        ],

        [
            (f * g - d * i) / det,
            (a * i - c * g) / det,
            (c * d - a * f) / det
        ],

        [
            (d * h - e * g) / det,
            (b * g - a * h) / det,
            (a * e - b * d) / det
        ]
    ]


# ============================================================
# NUMERICAL INVERSE KINEMATICS
# ============================================================

def inverse_ik(
    target_x,
    target_y,
    target_z
):

    # --------------------------------------------------------
    # Initial joint configuration
    # --------------------------------------------------------

    q = [
        0.0,
        0.0,
        0.0,
        0.0
    ]

    # --------------------------------------------------------
    # Joint limits
    # --------------------------------------------------------

    limits = [

        (
            -0.9 * math.pi,
             0.9 * math.pi
        ),

        (
            -0.57 * math.pi,
             0.5 * math.pi
        ),

        (
            -0.3 * math.pi,
             0.44 * math.pi
        ),

        (
            -0.57 * math.pi,
             0.65 * math.pi
        )
    ]

    # --------------------------------------------------------
    # IK parameters
    # --------------------------------------------------------

    learning_rate = 0.5

    damping = 1e-4

    max_iterations = 1000

    tolerance = 1e-5

    # --------------------------------------------------------
    # Iterative numerical IK
    # --------------------------------------------------------

    for iteration in range(max_iterations):

        x, y, z = forward_kinematics(*q)

        error = [

            target_x - x,

            target_y - y,

            target_z - z
        ]

        error_norm = math.sqrt(

            error[0] ** 2
            +
            error[1] ** 2
            +
            error[2] ** 2
        )

        # ----------------------------------------------------
        # Convergence
        # ----------------------------------------------------

        if error_norm < tolerance:

            return (
                q,
                iteration,
                error_norm
            )

        # ----------------------------------------------------
        # Numerical Jacobian
        # ----------------------------------------------------

        J = numerical_jacobian(q)

        JT = transpose(J)

        # ----------------------------------------------------
        # Damped Least Squares
        #
        # J+ = JT (J JT + λI)^-1
        # ----------------------------------------------------

        JJT = matmul_general(
            J,
            JT
        )

        JJT = add_damping(
            JJT,
            damping
        )

        JJT_inv = inverse_3x3(
            JJT
        )

        pseudo_inverse = matmul_general(
            JT,
            JJT_inv
        )

        # ----------------------------------------------------
        # Calculate joint change
        # ----------------------------------------------------

        dq = matvec_general(
            pseudo_inverse,
            error
        )

        # ----------------------------------------------------
        # Update joint angles
        # ----------------------------------------------------

        for i in range(4):

            q[i] += (
                learning_rate * dq[i]
            )

            # Apply joint limits

            q[i] = max(
                limits[i][0],
                min(
                    limits[i][1],
                    q[i]
                )
            )

    # --------------------------------------------------------
    # Maximum iterations reached
    # --------------------------------------------------------

    return (
        q,
        max_iterations,
        error_norm
    )


# ============================================================
# ROS 2 IK NODE
# ============================================================

class IKController(Node):

    def __init__(self):

        super().__init__(
            'ik_controller'
        )

        # ====================================================
        # IK → CANopen
        #
        # The CANopen command node subscribes to this topic.
        # ====================================================

        self.joint_angle_publisher = \
            self.create_publisher(
                JointState,
                '/ik/joint_angles',
                10
            )

        # ====================================================
        # TARGET POSITION
        # ====================================================

        self.target_x = None
        self.target_y = None
        self.target_z = None

        # ====================================================
        # COMMAND JOINT ANGLES
        #
        # These are the angles calculated by IK.
        # ====================================================

        self.commanded_q = [
            0.0,
            0.0,
            0.0,
            0.0
        ]

        # ====================================================
        # GAZEBO END-EFFECTOR FEEDBACK
        #
        # IMPORTANT:
        #
        # /gazebo/end_effector_position
        # is geometry_msgs/Point.
        #
        # Therefore we store XYZ directly.
        # ====================================================

        self.gazebo_x = None
        self.gazebo_y = None
        self.gazebo_z = None

        self.feedback_received = False

        # ====================================================
        # GAZEBO FEEDBACK SUBSCRIBER
        #
        # Gazebo feedback reader publishes:
        #
        # /gazebo/end_effector_position
        #
        # Message:
        #
        # geometry_msgs/Point
        # ====================================================

        self.feedback_subscriber = \
            self.create_subscription(
                Point,
                '/gazebo/end_effector_position',
                self.ee_feedback_callback,
                10
            )

        # ====================================================
        # FEEDBACK TIMER
        #
        # Compare target and actual position every 0.5 sec.
        # ====================================================

        self.feedback_timer = \
            self.create_timer(
                0.5,
                self.check_feedback
            )

        # ====================================================
        # STARTUP INFORMATION
        # ====================================================

        self.get_logger().info(
            '============================================'
        )

        self.get_logger().info(
            '          IK CONTROLLER STARTED'
        )

        self.get_logger().info(
            '============================================'
        )

        self.get_logger().info(
            'IK output topic:'
        )

        self.get_logger().info(
            '/ik/joint_angles'
        )

        self.get_logger().info(
            'Gazebo feedback topic:'
        )

        self.get_logger().info(
            '/gazebo/end_effector_position'
        )

        self.get_logger().info(
            'Feedback type: geometry_msgs/Point'
        )

    # ========================================================
    # GAZEBO END-EFFECTOR FEEDBACK CALLBACK
    # ========================================================

    def ee_feedback_callback(self, msg):

        # ----------------------------------------------------
        # Store ACTUAL Gazebo position
        # ----------------------------------------------------

        self.gazebo_x = msg.x
        self.gazebo_y = msg.y
        self.gazebo_z = msg.z

        self.feedback_received = True

    # ========================================================
    # PUBLISH IK JOINT ANGLES
    # ========================================================

    def publish_joint_angles(
        self,
        q
    ):

        msg = JointState()

        msg.name = [

            'joint1',
            'joint2',
            'joint3',
            'joint4'
        ]

        msg.position = [

            q[0],
            q[1],
            q[2],
            q[3]
        ]

        # ----------------------------------------------------
        # Publish
        # ----------------------------------------------------

        self.joint_angle_publisher.publish(
            msg
        )

        # ----------------------------------------------------
        # Store commanded angles
        # ----------------------------------------------------

        self.commanded_q = q.copy()

        # ----------------------------------------------------
        # Print
        # ----------------------------------------------------

        self.get_logger().info(
            '--------------------------------------------'
        )

        self.get_logger().info(
            'IK COMMAND PUBLISHED'
        )

        self.get_logger().info(
            f'joint1 = '
            f'{q[0]:.6f} rad '
            f'({math.degrees(q[0]):.3f} deg)'
        )

        self.get_logger().info(
            f'joint2 = '
            f'{q[1]:.6f} rad '
            f'({math.degrees(q[1]):.3f} deg)'
        )

        self.get_logger().info(
            f'joint3 = '
            f'{q[2]:.6f} rad '
            f'({math.degrees(q[2]):.3f} deg)'
        )

        self.get_logger().info(
            f'joint4 = '
            f'{q[3]:.6f} rad '
            f'({math.degrees(q[3]):.3f} deg)'
        )

        self.get_logger().info(
            '--------------------------------------------'
        )

    # ========================================================
    # COMPARE TARGET WITH ACTUAL GAZEBO POSITION
    # ========================================================

    def check_feedback(self):

        # ----------------------------------------------------
        # Target has not been entered yet.
        # ----------------------------------------------------

        if self.target_x is None:

            return

        # ----------------------------------------------------
        # Gazebo has not sent feedback yet.
        # ----------------------------------------------------

        if not self.feedback_received:

            self.get_logger().info(
                'Waiting for Gazebo end-effector feedback...'
            )

            return

        # ====================================================
        # POSITION ERROR
        #
        # Target XYZ - Actual Gazebo XYZ
        # ====================================================

        error_x = \
            self.target_x - self.gazebo_x

        error_y = \
            self.target_y - self.gazebo_y

        error_z = \
            self.target_z - self.gazebo_z

        position_error = math.sqrt(

            error_x ** 2
            +
            error_y ** 2
            +
            error_z ** 2
        )

        # ====================================================
        # PRINT FEEDBACK
        # ====================================================

        self.get_logger().info(
            ''
        )

        self.get_logger().info(
            '============================================'
        )

        self.get_logger().info(
            '          GAZEBO POSITION FEEDBACK'
        )

        self.get_logger().info(
            '============================================'
        )

        # ----------------------------------------------------
        # Target
        # ----------------------------------------------------

        self.get_logger().info(
            'TARGET POSITION'
        )

        self.get_logger().info(
            f'X = {self.target_x:.6f} m'
        )

        self.get_logger().info(
            f'Y = {self.target_y:.6f} m'
        )

        self.get_logger().info(
            f'Z = {self.target_z:.6f} m'
        )

        # ----------------------------------------------------
        # Actual Gazebo position
        # ----------------------------------------------------

        self.get_logger().info(
            '--------------------------------------------'
        )

        self.get_logger().info(
            'ACTUAL GAZEBO POSITION'
        )

        self.get_logger().info(
            f'X = {self.gazebo_x:.6f} m'
        )

        self.get_logger().info(
            f'Y = {self.gazebo_y:.6f} m'
        )

        self.get_logger().info(
            f'Z = {self.gazebo_z:.6f} m'
        )

        # ----------------------------------------------------
        # Position error
        # ----------------------------------------------------

        self.get_logger().info(
            '--------------------------------------------'
        )

        self.get_logger().info(
            'POSITION ERROR'
        )

        self.get_logger().info(
            f'X error = {error_x:.6f} m'
        )

        self.get_logger().info(
            f'Y error = {error_y:.6f} m'
        )

        self.get_logger().info(
            f'Z error = {error_z:.6f} m'
        )

        self.get_logger().info(
            f'Total error = '
            f'{position_error:.6f} m'
        )

        # ----------------------------------------------------
        # Status
        # ----------------------------------------------------

        tolerance = 0.001  # 1 mm

        if position_error <= tolerance:

            self.get_logger().info(
                'STATUS: TARGET REACHED '
                '(within 1 mm)'
            )

        else:

            self.get_logger().info(
                'STATUS: POSITION ERROR '
                f'{position_error:.6f} m'
            )

        self.get_logger().info(
            '============================================'
        )

    # ========================================================
    # SHUTDOWN
    # ========================================================

    def stop(self):

        self.get_logger().info(
            'IK controller stopped.'
        )


# ============================================================
# MAIN
# ============================================================

def main(args=None):

    rclpy.init(
        args=args
    )

    node = IKController()

    try:

        # ====================================================
        # GET TARGET POSITION
        # ====================================================

        target_x = float(
            input('X (m): ')
        )

        target_y = float(
            input('Y (m): ')
        )

        target_z = float(
            input('Z (m): ')
        )

        # ----------------------------------------------------
        # Store target
        # ----------------------------------------------------

        node.target_x = target_x
        node.target_y = target_y
        node.target_z = target_z

        print()

        print(
            '============================================'
        )

        print(
            '              NUMERICAL IK'
        )

        print(
            '============================================'
        )

        print()

        print(
            'TARGET END-EFFECTOR POSITION'
        )

        print(
            f'X = {target_x:.6f} m'
        )

        print(
            f'Y = {target_y:.6f} m'
        )

        print(
            f'Z = {target_z:.6f} m'
        )

        # ====================================================
        # RUN IK
        # ====================================================

        print()

        print(
            'Running Numerical IK...'
        )

        q, iterations, error = inverse_ik(

            target_x,

            target_y,

            target_z
        )

        # ====================================================
        # IK RESULT
        # ====================================================

        print()

        print(
            '============================================'
        )

        print(
            'IK SOLUTION'
        )

        print(
            '============================================'
        )

        print(
            f'q1 = {q[0]:.6f} rad '
            f'({math.degrees(q[0]):.3f} deg)'
        )

        print(
            f'q2 = {q[1]:.6f} rad '
            f'({math.degrees(q[1]):.3f} deg)'
        )

        print(
            f'q3 = {q[2]:.6f} rad '
            f'({math.degrees(q[2]):.3f} deg)'
        )

        print(
            f'q4 = {q[3]:.6f} rad '
            f'({math.degrees(q[3]):.3f} deg)'
        )

        print()

        print(
            f'IK Iterations = {iterations}'
        )

        print(
            f'IK Position Error = '
            f'{error:.8f} m'
        )

        # ====================================================
        # FK VERIFICATION
        #
        # This verifies the mathematical IK solution.
        #
        # It is NOT Gazebo feedback.
        # ====================================================

        fk_x, fk_y, fk_z = \
            forward_kinematics(*q)

        print()

        print(
            '============================================'
        )

        print(
            'MATHEMATICAL FK VERIFICATION'
        )

        print(
            '============================================'
        )

        print(
            f'X = {fk_x:.6f} m'
        )

        print(
            f'Y = {fk_y:.6f} m'
        )

        print(
            f'Z = {fk_z:.6f} m'
        )

        print()

        print(
            'Mathematical FK error:'
        )

        print(
            f'X error = '
            f'{target_x - fk_x:.8f} m'
        )

        print(
            f'Y error = '
            f'{target_y - fk_y:.8f} m'
        )

        print(
            f'Z error = '
            f'{target_z - fk_z:.8f} m'
        )

        # ====================================================
        # PUBLISH IK COMMAND
        # ====================================================

        print()

        print(
            'Publishing IK joint angles...'
        )

        node.publish_joint_angles(
            q
        )

        # ====================================================
        # PIPELINE INFORMATION
        # ====================================================

        print()

        print(
            '============================================'
        )

        print(
            '           COMMAND PIPELINE'
        )

        print(
            '============================================'
        )

        print(
            'Target XYZ'
        )

        print(
            '   ↓'
        )

        print(
            'IK Node'
        )

        print(
            '   ↓ q1 q2 q3 q4'
        )

        print(
            'CANopen Command Node'
        )

        print(
            '   ↓'
        )

        print(
            'vcan0'
        )

        print(
            '   ↓'
        )

        print(
            'CAN Angle Decoder'
        )

        print(
            '   ↓'
        )

        print(
            'Gazebo'
        )

        # ====================================================
        # FEEDBACK PIPELINE
        # ====================================================

        print()

        print(
            '============================================'
        )

        print(
            '           FEEDBACK PIPELINE'
        )

        print(
            '============================================'
        )

        print(
            'Gazebo'
        )

        print(
            '   ↓ actual joint states'
        )

        print(
            '/joint_states'
        )

        print(
            '   ↓'
        )

        print(
            'gazebo_feedback_reader'
        )

        print(
            '   ↓ actual EE XYZ'
        )

        print(
            '/gazebo/end_effector_position'
        )

        print(
            '   ↓'
        )

        print(
            'IK Node'
        )

        print(
            '   ↓'
        )

        print(
            'Target XYZ vs Actual XYZ'
        )

        print()

        print(
            '============================================'
        )

        print(
            'IK COMMAND SENT'
        )

        print(
            'Waiting for Gazebo feedback...'
        )

        print(
            '============================================'
        )

        # ====================================================
        # KEEP NODE ALIVE
        #
        # This is necessary because the node must continue
        # receiving Gazebo feedback.
        # ====================================================

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        print()

        print(
            'IK node stopped.'
        )

    except ValueError as e:

        node.get_logger().error(
            f'Invalid input: {e}'
        )

    finally:

        node.stop()

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


# ============================================================
# ENTRY POINT
# ============================================================

if __name__ == '__main__':

    main()
