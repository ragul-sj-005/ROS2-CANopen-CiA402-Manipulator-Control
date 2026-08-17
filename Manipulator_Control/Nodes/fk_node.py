import math


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


def forward_kinematics(q1, q2, q3, q4):

    # ---------------------------------------------------------
    # Actual OpenMANIPULATOR-X geometry from URDF
    # ---------------------------------------------------------

    # base_link -> link1
    base_height = 0.05

    # joint1 origin: link1 -> link2
    joint1_offset = [0.012, 0.0, 0.017]

    # joint2 origin: link2 -> link3
    joint2_offset = [0.0, 0.0, 0.0595]

    # joint3 origin: link3 -> link4
    joint3_offset = [0.024, 0.0, 0.128]

    # joint4 origin: link4 -> link5
    joint4_offset = [0.124, 0.0, 0.0]

    # link5 -> end_effector_link
    ee_offset = [0.126, 0.0, 0.0]

    # ---------------------------------------------------------
    # Rotation chain
    # ---------------------------------------------------------

    R1 = rot_z(q1)
    R2 = rot_y(q2)
    R3 = rot_y(q3)
    R4 = rot_y(q4)

    # ---------------------------------------------------------
    # Position calculation
    # ---------------------------------------------------------

    # Start at base_link origin
    position = [0.0, 0.0, base_height]

    # link1 -> joint1
    offset = matvec(R1, joint1_offset)
    position = [
        position[i] + offset[i]
        for i in range(3)
    ]

    # joint1 rotation
    R = R1

    # joint2 offset
    offset = matvec(R, joint2_offset)
    position = [
        position[i] + offset[i]
        for i in range(3)
    ]

    # joint2 rotation
    R = matmul(R, R2)

    # joint3 offset
    offset = matvec(R, joint3_offset)
    position = [
        position[i] + offset[i]
        for i in range(3)
    ]

    # joint3 rotation
    R = matmul(R, R3)

    # joint4 offset
    offset = matvec(R, joint4_offset)
    position = [
        position[i] + offset[i]
        for i in range(3)
    ]

    # joint4 rotation
    R = matmul(R, R4)

    # End-effector offset
    offset = matvec(R, ee_offset)
    position = [
        position[i] + offset[i]
        for i in range(3)
    ]

    return position[0], position[1], position[2]


def main():

    # ---------------------------------------------------------
    # Test configuration
    # ---------------------------------------------------------

    q1 = 0.2
    q2 = 0.2
    q3 = 0.1
    q4 = 0.1

    x, y, z = forward_kinematics(q1, q2, q3, q4)

    print("Joint angles:")
    print(f"q1 = {q1:.4f} rad")
    print(f"q2 = {q2:.4f} rad")
    print(f"q3 = {q3:.4f} rad")
    print(f"q4 = {q4:.4f} rad")

    print("\nCalculated End-Effector Position:")
    print(f"X = {x:.4f} m")
    print(f"Y = {y:.4f} m")
    print(f"Z = {z:.4f} m")


if __name__ == '__main__':
    main()
