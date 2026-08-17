import math

import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState


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


def matmul(A, B):
    return [
        [
            sum(A[i][k] * B[k][j] for k in range(3))
            for j in range(3)
        ]
        for i in range(3)
    ]


def matvec(R, v):
    return [
        sum(R[i][j] * v[j] for j in range(3))
        for i in range(3)
    ]


# ============================================================
# FORWARD KINEMATICS
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

    position = [0.0, 0.0, base_height]

    offset = matvec(R1, joint1_offset)
    position = [
        position[i] + offset[i]
        for i in range(3)
    ]

    R = R1

    offset = matvec(R, joint2_offset)
    position = [
        position[i] + offset[i]
        for i in range(3)
    ]

    R = matmul(R, R2)

    offset = matvec(R, joint3_offset)
    position = [
        position[i] + offset[i]
        for i in range(3)
    ]

    R = matmul(R, R3)

    offset = matvec(R, joint4_offset)
    position = [
        position[i] + offset[i]
        for i in range(3)
    ]

    R = matmul(R, R4)

    offset = matvec(R, ee_offset)
    position = [
        position[i] + offset[i]
        for i in range(3)
    ]

    return position[0], position[1], position[2]


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
        [J[j][i] for j in range(4)]
        for i in range(3)
    ]


# ============================================================
# MATRIX FUNCTIONS
# ============================================================

def transpose(A):

    return [
        [A[j][i] for j in range(len(A))]
        for i in range(len(A[0]))
    ]


def matmul_general(A, B):

    return [
        [
            sum(A[i][k] * B[k][j] for k in range(len(B)))
            for j in range(len(B[0]))
        ]
        for i in range(len(A))
    ]


def matvec_general(A, v):

    return [
        sum(A[i][j] * v[j] for j in range(len(v)))
        for i in range(len(A))
    ]


def add_damping(A, damping):

    result = [row[:] for row in A]

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
        raise ValueError("Matrix is singular.")

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

def inverse_ik(target_x, target_y, target_z):

    q = [0.0, 0.0, 0.0, 0.0]

    limits = [
        (-0.9 * math.pi,  0.9 * math.pi),
        (-0.57 * math.pi, 0.5 * math.pi),
        (-0.3 * math.pi,  0.44 * math.pi),
        (-0.57 * math.pi, 0.65 * math.pi)
    ]

    learning_rate = 0.5
    damping = 1e-4

    max_iterations = 1000
    tolerance = 1e-5

    for iteration in range(max_iterations):

        x, y, z = forward_kinematics(*q)

        error = [
            target_x - x,
            target_y - y,
            target_z - z
        ]

        error_norm = math.sqrt(
            error[0] ** 2 +
            error[1] ** 2 +
            error[2] ** 2
        )

        if error_norm < tolerance:
            return q, iteration, error_norm

        J = numerical_jacobian(q)

        JT = transpose(J)

        JJT = matmul_general(J, JT)

        JJT = add_damping(JJT, damping)

        JJT_inv = inverse_3x3(JJT)

        pseudo_inverse = matmul_general(
            JT,
            JJT_inv
        )

        dq = matvec_general(
            pseudo_inverse,
            error
        )

        for i in range(4):

            q[i] += learning_rate * dq[i]

            q[i] = max(
                limits[i][0],
                min(limits[i][1], q[i])
            )

    return q, max_iterations, error_norm


# ============================================================
# ROS 2 IK + ARM CONTROLLER NODE
# ============================================================

class IKController(Node):

    def __init__(self):

        super().__init__('ik_controller')

        self.trajectory_publisher = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )

        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.latest_joint_states = {}

        self.get_logger().info(
            'IK Controller started.'
        )


    def joint_state_callback(self, msg):

        for i, name in enumerate(msg.name):

            if i < len(msg.position):

                self.latest_joint_states[name] = msg.position[i]


    # ========================================================
    # SEND JOINT COMMAND
    # ========================================================

    def send_joint_command(self, q):

        trajectory = JointTrajectory()

        trajectory.joint_names = [
            'joint1',
            'joint2',
            'joint3',
            'joint4'
        ]

        point = JointTrajectoryPoint()

        point.positions = [
            q[0],
            q[1],
            q[2],
            q[3]
        ]

        point.time_from_start.sec = 3

        trajectory.points.append(point)

        self.trajectory_publisher.publish(
            trajectory
        )

        self.get_logger().info(
            'Joint trajectory command published.'
        )


    # ========================================================
    # GET ACTUAL JOINT POSITIONS
    # ========================================================

    def get_actual_joints(self):

        required_joints = [
            'joint1',
            'joint2',
            'joint3',
            'joint4'
        ]

        for joint in required_joints:

            if joint not in self.latest_joint_states:

                return None

        return [
            self.latest_joint_states['joint1'],
            self.latest_joint_states['joint2'],
            self.latest_joint_states['joint3'],
            self.latest_joint_states['joint4']
        ]


# ============================================================
# MAIN
# ============================================================

def main(args=None):

    rclpy.init(args=args)

    node = IKController()

    # --------------------------------------------------------
    # TARGET POSITION
    # --------------------------------------------------------

    target_x = float(input("X (m): "))
    target_y = float(input("Y (m): "))
    target_z = float(input("Z (m): "))

    print()
    print('========================================')
    print('     IK → ROS 2 → GAZEBO INTEGRATION')
    print('========================================')

    print()
    print('Target End-Effector Position:')
    print(f'X = {target_x:.4f} m')
    print(f'Y = {target_y:.4f} m')
    print(f'Z = {target_z:.4f} m')

    # --------------------------------------------------------
    # RUN IK
    # --------------------------------------------------------

    print()
    print('Running Numerical IK...')

    q, iterations, error = inverse_ik(
        target_x,
        target_y,
        target_z
    )

    print()
    print('IK Solution:')
    print(f'q1 = {q[0]:.6f} rad')
    print(f'q2 = {q[1]:.6f} rad')
    print(f'q3 = {q[2]:.6f} rad')
    print(f'q4 = {q[3]:.6f} rad')

    print()
    print(f'IK Iterations = {iterations}')
    print(f'IK Position Error = {error:.8f} m')

    # --------------------------------------------------------
    # FK VERIFICATION BEFORE COMMAND
    # --------------------------------------------------------

    fk_x, fk_y, fk_z = forward_kinematics(*q)

    print()
    print('FK Verification Before Gazebo:')
    print(f'X = {fk_x:.6f} m')
    print(f'Y = {fk_y:.6f} m')
    print(f'Z = {fk_z:.6f} m')

    # --------------------------------------------------------
    # WAIT FOR JOINT STATES
    # --------------------------------------------------------

    print()
    print('Waiting for /joint_states...')

    start_time = node.get_clock().now()

    while rclpy.ok():

        rclpy.spin_once(node, timeout_sec=0.1)

        actual = node.get_actual_joints()

        if actual is not None:
            break

        elapsed = (
            node.get_clock().now() - start_time
        ).nanoseconds / 1e9

        if elapsed > 10.0:

            print(
                'ERROR: Did not receive joint states.'
            )

            node.destroy_node()
            rclpy.shutdown()
            return

    print('Joint states received.')

    # --------------------------------------------------------
    # SEND IK SOLUTION TO GAZEBO
    # --------------------------------------------------------

    print()
    print('Sending IK solution to Gazebo...')

    node.send_joint_command(q)

    # --------------------------------------------------------
    # WAIT FOR ROBOT MOVEMENT
    # --------------------------------------------------------

    print()
    print('Waiting 4 seconds for manipulator movement...')

    start_time = node.get_clock().now()

    while rclpy.ok():

        rclpy.spin_once(node, timeout_sec=0.1)

        elapsed = (
            node.get_clock().now() - start_time
        ).nanoseconds / 1e9

        if elapsed >= 4.0:
            break

    # --------------------------------------------------------
    # READ ACTUAL JOINT STATES
    # --------------------------------------------------------

    actual_q = node.get_actual_joints()

    print()
    print('========================================')
    print('        JOINT POSITION RESULTS')
    print('========================================')

    if actual_q is None:

        print(
            'ERROR: Actual joint positions unavailable.'
        )

    else:

        joint_names = [
            'joint1',
            'joint2',
            'joint3',
            'joint4'
        ]

        max_joint_error = 0.0

        for i in range(4):

            joint_error = abs(
                q[i] - actual_q[i]
            )

            max_joint_error = max(
                max_joint_error,
                joint_error
            )

            print(
                f'{joint_names[i]}: '
                f'commanded = {q[i]:.6f}, '
                f'actual = {actual_q[i]:.6f}, '
                f'error = {joint_error:.6f} rad'
            )

        # ----------------------------------------------------
        # ACTUAL FK
        # ----------------------------------------------------

        actual_x, actual_y, actual_z = \
            forward_kinematics(*actual_q)

        position_error = math.sqrt(
            (target_x - actual_x) ** 2 +
            (target_y - actual_y) ** 2 +
            (target_z - actual_z) ** 2
        )

        print()
        print('========================================')
        print('       END-EFFECTOR RESULTS')
        print('========================================')

        print()
        print('Target Position:')
        print(
            f'X = {target_x:.6f} m\n'
            f'Y = {target_y:.6f} m\n'
            f'Z = {target_z:.6f} m'
        )

        print()
        print('Actual Position from FK:')
        print(
            f'X = {actual_x:.6f} m\n'
            f'Y = {actual_y:.6f} m\n'
            f'Z = {actual_z:.6f} m'
        )

        print()
        print(
            f'Final End-Effector Position Error = '
            f'{position_error:.8f} m'
        )

        print(
            f'Maximum Joint Error = '
            f'{max_joint_error:.8f} rad'
        )

        print()

        if position_error < 0.001:

            print(
                'STATUS: IK → GAZEBO → FK SUCCESS'
            )

        else:

            print(
                'STATUS: POSITION ERROR TOO HIGH'
            )

    print()
    print('========================================')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
