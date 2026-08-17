#!/usr/bin/env python3

import math
import time

import can
import rclpy
from rclpy.node import Node


class CANopenPositionReader(Node):

    def __init__(self):
        super().__init__('canopen_position_reader')

        # ---------------------------------------------------------
        # ROS PARAMETERS
        # ---------------------------------------------------------

        self.declare_parameter('can_interface', 'vcan0')

        # IMPORTANT:
        # This is a temporary/test value.
        # Replace it with the actual scaling of your drive.
        self.declare_parameter('counts_per_rev', 4096.0)

        self.declare_parameter('poll_rate', 5.0)

        self.interface = self.get_parameter(
            'can_interface'
        ).value

        self.counts_per_rev = float(
            self.get_parameter(
                'counts_per_rev'
            ).value
        )

        self.poll_rate = float(
            self.get_parameter(
                'poll_rate'
            ).value
        )

        # ---------------------------------------------------------
        # NODE CONFIGURATION
        # ---------------------------------------------------------

        self.nodes = {
            2: 'joint1',
            3: 'joint2',
            4: 'joint3',
            5: 'joint4'
        }

        # CANopen SDO COB-IDs
        #
        # Request:
        #   0x600 + Node ID
        #
        # Response:
        #   0x580 + Node ID

        # ---------------------------------------------------------
        # OPEN CAN INTERFACE
        # ---------------------------------------------------------

        try:

            self.bus = can.Bus(
                interface='socketcan',
                channel=self.interface,
                receive_own_messages=False
            )

            self.get_logger().info(
                f'CAN interface opened: {self.interface}'
            )

        except Exception as e:

            self.get_logger().error(
                f'Failed to open CAN interface {self.interface}: {e}'
            )

            raise

        # ---------------------------------------------------------
        # TIMER
        # ---------------------------------------------------------

        timer_period = 1.0 / self.poll_rate

        self.timer = self.create_timer(
            timer_period,
            self.read_all_positions
        )

        self.get_logger().info(
            '============================================'
        )

        self.get_logger().info(
            '       CANopen POSITION READER'
        )

        self.get_logger().info(
            '============================================'
        )

        self.get_logger().info(
            f'CAN interface    : {self.interface}'
        )

        self.get_logger().info(
            'Object           : 0x6064:00'
        )

        self.get_logger().info(
            'Meaning          : Position Actual Value'
        )

        self.get_logger().info(
            f'Counts / rev     : {self.counts_per_rev}'
        )

        self.get_logger().info(
            f'Poll rate        : {self.poll_rate} Hz'
        )

        self.get_logger().info(
            '============================================'
        )

    # =============================================================
    # READ ALL FOUR JOINT POSITIONS
    # =============================================================

    def read_all_positions(self):

        for node_id, joint_name in self.nodes.items():

            try:

                position = self.read_position(
                    node_id,
                    joint_name
                )

            except Exception as e:

                self.get_logger().error(
                    f'{joint_name} | Node {node_id} | '
                    f'Error: {e}'
                )

    # =============================================================
    # READ 0x6064:00
    # =============================================================

    def read_position(self, node_id, joint_name):

        # ---------------------------------------------------------
        # CANopen SDO request
        #
        # 40 = Initiate SDO upload/read
        # 64 60 = Index 0x6064
        # 00 = Sub-index 0x00
        # ---------------------------------------------------------

        request_cob_id = 0x600 + node_id

        response_cob_id = 0x580 + node_id

        data = [
            0x40,
            0x64,
            0x60,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00
        ]

        message = can.Message(
            arbitration_id=request_cob_id,
            data=data,
            is_extended_id=False
        )

        # ---------------------------------------------------------
        # SEND REQUEST
        # ---------------------------------------------------------

        self.bus.send(message)

        # ---------------------------------------------------------
        # WAIT FOR RESPONSE
        # ---------------------------------------------------------

        response = self.bus.recv(timeout=0.5)

        if response is None:

            raise TimeoutError(
                f'No response from Node {node_id}'
            )

        # ---------------------------------------------------------
        # CHECK CAN ID
        # ---------------------------------------------------------

        if response.arbitration_id != response_cob_id:

            raise RuntimeError(
                f'Unexpected CAN ID: '
                f'0x{response.arbitration_id:03X}'
            )

        # ---------------------------------------------------------
        # CHECK RESPONSE LENGTH
        # ---------------------------------------------------------

        if len(response.data) < 8:

            raise RuntimeError(
                f'Invalid response length: '
                f'{len(response.data)}'
            )

        # ---------------------------------------------------------
        # CHECK SDO RESPONSE COMMAND BYTE
        #
        # For a normal 4-byte expedited upload:
        #
        # 43 = 4-byte data response
        # ---------------------------------------------------------

        command = response.data[0]

        if command != 0x43:

            # Abort response starts with 0x80

            if command == 0x80:

                abort_code = int.from_bytes(
                    response.data[4:8],
                    byteorder='little',
                    signed=False
                )

                raise RuntimeError(
                    f'SDO Abort: '
                    f'0x{abort_code:08X}'
                )

            raise RuntimeError(
                f'Unexpected SDO response command: '
                f'0x{command:02X}'
            )

        # ---------------------------------------------------------
        # CHECK THAT RESPONSE IS FOR 0x6064:00
        # ---------------------------------------------------------

        index = (
            response.data[1]
            | (response.data[2] << 8)
        )

        subindex = response.data[3]

        if index != 0x6064 or subindex != 0x00:

            raise RuntimeError(
                f'Unexpected object: '
                f'0x{index:04X}:{subindex:02X}'
            )

        # ---------------------------------------------------------
        # EXTRACT POSITION
        #
        # Bytes:
        #
        # data[4] = LSB
        # data[5]
        # data[6]
        # data[7] = MSB
        #
        # CANopen uses little-endian representation.
        #
        # Position Actual Value is a signed 32-bit value.
        # ---------------------------------------------------------

        raw_bytes = bytes(response.data[4:8])

        raw_position = int.from_bytes(
            raw_bytes,
            byteorder='little',
            signed=True
        )

        # ---------------------------------------------------------
        # CONVERT COUNTS → REVOLUTIONS
        # ---------------------------------------------------------

        revolutions = (
            raw_position /
            self.counts_per_rev
        )

        # ---------------------------------------------------------
        # CONVERT REVOLUTIONS → RADIANS
        # ---------------------------------------------------------

        angle_rad = (
            revolutions *
            2.0 *
            math.pi
        )

        # ---------------------------------------------------------
        # CONVERT RADIANS → DEGREES
        # ---------------------------------------------------------

        angle_deg = math.degrees(
            angle_rad
        )

        # ---------------------------------------------------------
        # PRINT RESULT
        # ---------------------------------------------------------

        self.print_position(
            node_id=node_id,
            joint_name=joint_name,
            request=message,
            response=response,
            raw_position=raw_position,
            revolutions=revolutions,
            angle_rad=angle_rad,
            angle_deg=angle_deg
        )

        return angle_rad

    # =============================================================
    # PRINT POSITION INFORMATION
    # =============================================================

    def print_position(
        self,
        node_id,
        joint_name,
        request,
        response,
        raw_position,
        revolutions,
        angle_rad,
        angle_deg
    ):

        tx_data = ' '.join(
            f'{byte:02X}'
            for byte in request.data
        )

        rx_data = ' '.join(
            f'{byte:02X}'
            for byte in response.data
        )

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
            f'CAN TX       : '
            f'0x{request.arbitration_id:03X} '
            f'[{tx_data}]'
        )

        self.get_logger().info(
            f'CAN RX       : '
            f'0x{response.arbitration_id:03X} '
            f'[{rx_data}]'
        )

        self.get_logger().info(
            'Object       : 0x6064:00'
        )

        self.get_logger().info(
            'Meaning      : Position Actual Value'
        )

        self.get_logger().info(
            f'Raw position : {raw_position} counts'
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

    # =============================================================
    # SHUTDOWN
    # =============================================================

    def destroy_node(self):

        self.get_logger().info(
            'CANopen position reader stopped.'
        )

        try:

            self.bus.shutdown()

        except Exception:

            pass

        super().destroy_node()


# =================================================================
# MAIN
# =================================================================

def main(args=None):

    rclpy.init(args=args)

    node = None

    try:

        node = CANopenPositionReader()

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        if node is not None:

            node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == '__main__':

    main()
