#!/usr/bin/env python3

import math
import can

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class CANopenAngleDecoder(Node):

    def __init__(self):
        super().__init__('canopen_angle_decoder')

        # ---------------------------------------------------------
        # CAN configuration
        # ---------------------------------------------------------
        self.CAN_INTERFACE = 'vcan0'

        # CANopen SDO request IDs
        # Node 2 -> 0x602
        # Node 3 -> 0x603
        # Node 4 -> 0x604
        # Node 5 -> 0x605
        self.node_ids = {
            0x602: 2,
            0x603: 3,
            0x604: 4,
            0x605: 5,
        }

        # Joint mapping
        self.joint_names = {
            2: 'joint1',
            3: 'joint2',
            4: 'joint3',
            5: 'joint4',
        }

        # ---------------------------------------------------------
        # IMPORTANT:
        #
        # This value MUST match the encoder/drive scaling used
        # by your CANopen system.
        #
        # Example:
        # 100000 counts = 1 revolution
        #
        # Change this after confirming the actual drive scaling.
        # ---------------------------------------------------------
        self.counts_per_revolution = 100000.0

        # ---------------------------------------------------------
        # ROS publisher
        #
        # [joint1, joint2, joint3, joint4]
        # values are radians
        # ---------------------------------------------------------
        self.angle_pub = self.create_publisher(
            Float64MultiArray,
            '/decoded_joint_angles',
            10
        )

        # ---------------------------------------------------------
        # Open CAN bus
        # ---------------------------------------------------------
        try:
            self.bus = can.interface.Bus(
                channel=self.CAN_INTERFACE,
                interface='socketcan'
            )

            self.get_logger().info(
                '============================================'
            )
            self.get_logger().info(
                '       CANopen ANGLE DECODER'
            )
            self.get_logger().info(
                '============================================'
            )
            self.get_logger().info(
                f'CAN interface : {self.CAN_INTERFACE}'
            )
            self.get_logger().info(
                'Listening for SDO target-position frames:'
            )
            self.get_logger().info(
                '  Node 2 / joint1 : 0x602'
            )
            self.get_logger().info(
                '  Node 3 / joint2 : 0x603'
            )
            self.get_logger().info(
                '  Node 4 / joint3 : 0x604'
            )
            self.get_logger().info(
                '  Node 5 / joint4 : 0x605'
            )
            self.get_logger().info(
                'Object: 0x607A:00 Target Position'
            )
            self.get_logger().info(
                f'Counts/revolution: '
                f'{self.counts_per_revolution}'
            )
            self.get_logger().info(
                '============================================'
            )

        except Exception as e:
            self.get_logger().error(
                f'Failed to open CAN interface: {e}'
            )
            raise

        # Store latest decoded angles
        self.joint_angles = {
            2: 0.0,
            3: 0.0,
            4: 0.0,
            5: 0.0,
        }

        # ---------------------------------------------------------
        # Timer
        #
        # Periodically checks vcan0 for CAN frames.
        # ---------------------------------------------------------
        self.timer = self.create_timer(
            0.01,
            self.read_can_frames
        )

    # =============================================================
    # CAN FRAME READER
    # =============================================================

    def read_can_frames(self):

        while True:

            try:
                message = self.bus.recv(timeout=0.0)

            except Exception as e:
                self.get_logger().error(
                    f'CAN receive error: {e}'
                )
                return

            if message is None:
                break

            self.decode_frame(message)

    # =============================================================
    # CANopen SDO FRAME DECODER
    # =============================================================

    def decode_frame(self, message):

        can_id = message.arbitration_id

        # We only care about:
        #
        # 0x602
        # 0x603
        # 0x604
        # 0x605
        #
        if can_id not in self.node_ids:
            return

        node_id = self.node_ids[can_id]

        data = list(message.data)

        # CANopen SDO frame should contain 8 bytes
        if len(data) < 8:
            self.get_logger().warning(
                f'Invalid CAN frame length: '
                f'ID=0x{can_id:03X}'
            )
            return

        # ---------------------------------------------------------
        # Check whether this is a WRITE command to 0x607A:00
        #
        # SDO expedited 4-byte write:
        #
        # Byte 0 = 0x23
        # Byte 1 = Index low  (0x7A)
        # Byte 2 = Index high (0x60)
        # Byte 3 = Subindex   (0x00)
        # Byte 4-7 = value
        #
        # Example:
        #
        # 23 7A 60 00 6F 01 00 00
        #
        # means:
        #
        # Target Position = 0x0000016F
        #                   = 367 counts
        # ---------------------------------------------------------

        command = data[0]

        index_low = data[1]
        index_high = data[2]
        subindex = data[3]

        index = index_low | (index_high << 8)

        # Target Position = 0x607A:00
        if (
            command == 0x23
            and index == 0x607A
            and subindex == 0x00
        ):

            # -----------------------------------------------------
            # Extract 4-byte signed integer
            #
            # CANopen uses little-endian byte order.
            # -----------------------------------------------------

            raw_count = int.from_bytes(
                bytes(data[4:8]),
                byteorder='little',
                signed=True
            )

            # -----------------------------------------------------
            # counts -> revolutions
            # -----------------------------------------------------

            revolutions = (
                raw_count /
                self.counts_per_revolution
            )

            # -----------------------------------------------------
            # revolutions -> radians
            # -----------------------------------------------------

            angle_rad = revolutions * 2.0 * math.pi

            # -----------------------------------------------------
            # revolutions -> degrees
            # -----------------------------------------------------

            angle_deg = math.degrees(angle_rad)

            # Save latest value
            self.joint_angles[node_id] = angle_rad

            joint_name = self.joint_names[node_id]

            # -----------------------------------------------------
            # Print decoded information
            # -----------------------------------------------------

            self.get_logger().info(
                ''
            )

            self.get_logger().info(
                '--------------------------------------------'
            )

            self.get_logger().info(
                f'{joint_name} | Node {node_id}'
            )

            self.get_logger().info(
                f'CAN ID       : 0x{can_id:03X}'
            )

            self.get_logger().info(
                'CAN DATA     : '
                + ' '.join(
                    f'{byte:02X}' for byte in data
                )
            )

            self.get_logger().info(
                'Object       : 0x607A:00'
            )

            self.get_logger().info(
                'Meaning      : Target Position'
            )

            self.get_logger().info(
                f'Raw counts   : {raw_count}'
            )

            self.get_logger().info(
                f'Position     : {revolutions:.6f} rev'
            )

            self.get_logger().info(
                f'Angle        : {angle_rad:.6f} rad'
            )

            self.get_logger().info(
                f'Angle        : {angle_deg:.3f} deg'
            )

            self.get_logger().info(
                '--------------------------------------------'
            )

            # -----------------------------------------------------
            # Publish all four joint angles
            # -----------------------------------------------------

            msg = Float64MultiArray()

            msg.data = [
                self.joint_angles[2],
                self.joint_angles[3],
                self.joint_angles[4],
                self.joint_angles[5],
            ]

            self.angle_pub.publish(msg)

            self.get_logger().info(
                'Published /decoded_joint_angles: '
                f'['
                f'{msg.data[0]:.6f}, '
                f'{msg.data[1]:.6f}, '
                f'{msg.data[2]:.6f}, '
                f'{msg.data[3]:.6f}'
                f'] rad'
            )

    # =============================================================
    # SHUTDOWN
    # =============================================================

    def destroy_node(self):

        self.get_logger().info(
            'CANopen angle decoder stopped.'
        )

        try:
            self.bus.shutdown()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = CANopenAngleDecoder()

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
