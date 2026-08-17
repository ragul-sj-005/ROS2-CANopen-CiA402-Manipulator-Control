#!/usr/bin/env python3

import socket
import struct
import time
import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState


class CANopenCommandNode(Node):

    def __init__(self):

        super().__init__("canopen_command_node")

        # =========================================================
        # CANopen configuration
        # =========================================================

        self.interface = "vcan0"

        # Joint -> CANopen Node ID
        self.joint_to_node = {
            "joint1": 2,
            "joint2": 3,
            "joint3": 4,
            "joint4": 5,
        }

        # =========================================================
        # Position scaling
        # =========================================================
        #
        # Assumption:
        # 100000 encoder counts = 1 mechanical revolution
        #
        # Therefore:
        #
        # counts_per_radian =
        #     100000 / (2*pi)
        #
        # radians -> counts:
        # counts = radians * counts_per_radian
        #
        # IMPORTANT:
        # This value must match the actual CANopen drive/
        # encoder/gear-ratio scaling.
        #
        # =========================================================

        self.counts_per_radian = 100000.0 / (2.0 * math.pi)

        # =========================================================
        # SocketCAN
        # =========================================================

        self.sock = socket.socket(
            socket.AF_CAN,
            socket.SOCK_RAW,
            socket.CAN_RAW
        )

        self.sock.bind((self.interface,))

        # =========================================================
        # ROS 2 subscriber
        # =========================================================

        self.subscription = self.create_subscription(
            JointState,
            "/ik/joint_angles",
            self.joint_angle_callback,
            10
        )

        # =========================================================
        # Startup information
        # =========================================================

        self.get_logger().info(
            "============================================"
        )

        self.get_logger().info(
            "        CANopen COMMAND NODE"
        )

        self.get_logger().info(
            "============================================"
        )

        self.get_logger().info(
            f"CAN interface : {self.interface}"
        )

        self.get_logger().info(
            "Subscribed topic:"
        )

        self.get_logger().info(
            "  /ik/joint_angles"
        )

        self.get_logger().info(
            "Joint -> CANopen Node:"
        )

        for joint, node_id in self.joint_to_node.items():

            self.get_logger().info(
                f"  {joint} -> Node {node_id}"
            )

        self.get_logger().info(
            "============================================"
        )

    # =============================================================
    # JOINT ANGLE CALLBACK
    # =============================================================

    def joint_angle_callback(self, msg):

        self.get_logger().info("")

        self.get_logger().info(
            "============================================"
        )

        self.get_logger().info(
            "       IK JOINT ANGLES RECEIVED"
        )

        self.get_logger().info(
            "============================================"
        )

        # ---------------------------------------------------------
        # Required joints
        # ---------------------------------------------------------

        required_joints = [
            "joint1",
            "joint2",
            "joint3",
            "joint4"
        ]

        joint_angles = {}

        # ---------------------------------------------------------
        # Extract joint positions
        # ---------------------------------------------------------

        for joint in required_joints:

            if joint not in msg.name:

                self.get_logger().error(
                    f"{joint} not found in JointState message."
                )

                return

            index = msg.name.index(joint)

            if index >= len(msg.position):

                self.get_logger().error(
                    f"No position value available for {joint}."
                )

                return

            joint_angles[joint] = msg.position[index]

        # ---------------------------------------------------------
        # Display received joint angles
        # ---------------------------------------------------------

        for joint in required_joints:

            self.get_logger().info(
                f"{joint}: "
                f"{joint_angles[joint]:.6f} rad"
            )

        # ---------------------------------------------------------
        # Send each joint command
        # ---------------------------------------------------------

        for joint in required_joints:

            node_id = self.joint_to_node[joint]

            angle = joint_angles[joint]

            # -----------------------------------------------------
            # Convert radians -> encoder counts
            # -----------------------------------------------------

            counts = round(
                angle * self.counts_per_radian
            )

            self.get_logger().info(
                f"{joint}: "
                f"{angle:.6f} rad -> "
                f"{counts} counts"
            )

            # -----------------------------------------------------
            # Send target position
            # -----------------------------------------------------

            success = self.send_target_position(
                node_id,
                counts
            )

            if not success:

                self.get_logger().error(
                    f"Failed to command "
                    f"{joint} (Node {node_id})"
                )

                return

            time.sleep(0.05)

        # ---------------------------------------------------------
        # All commands successful
        # ---------------------------------------------------------

        self.get_logger().info(
            "============================================"
        )

        self.get_logger().info(
            "All four CANopen position commands sent."
        )

        self.get_logger().info(
            "============================================"
        )

    # =============================================================
    # SEND SDO WRITE
    # =============================================================

    def send_sdo_write(
        self,
        node_id,
        index,
        subindex,
        value,
        data_size=4
    ):

        # ---------------------------------------------------------
        # CANopen SDO COB-IDs
        # ---------------------------------------------------------

        request_id = 0x600 + node_id
        response_id = 0x580 + node_id

        # ---------------------------------------------------------
        # Select expedited SDO command
        # ---------------------------------------------------------
        #
        # 2 bytes -> 0x2B
        # 4 bytes -> 0x23
        #
        # 0x6040 Controlword = UINT16 -> 2 bytes
        #
        # 0x607A Target Position = INT32 -> 4 bytes
        #
        # ---------------------------------------------------------

        if data_size == 2:

            command = 0x2B

            data = struct.pack(
                "<H",
                int(value)
            )

        elif data_size == 4:

            command = 0x23

            data = struct.pack(
                "<i",
                int(value)
            )

        else:

            raise ValueError(
                "Unsupported SDO data size. "
                "Use 2 or 4."
            )

        # ---------------------------------------------------------
        # Construct 8-byte CANopen SDO payload
        # ---------------------------------------------------------

        frame_data = struct.pack(
            "<BHB",
            command,
            index,
            subindex
        ) + data

        # ---------------------------------------------------------
        # Construct SocketCAN frame
        # ---------------------------------------------------------

        frame = struct.pack(
            "=IB3x8s",
            request_id,
            8,
            frame_data
        )

        # ---------------------------------------------------------
        # Log transmission
        # ---------------------------------------------------------

        self.get_logger().info(
            f"CAN TX -> "
            f"Node {node_id} | "
            f"ID 0x{request_id:03X} | "
            f"Object 0x{index:04X}:{subindex:02X} | "
            f"Value {value} | "
            f"Size {data_size}"
        )

        # ---------------------------------------------------------
        # Send CAN frame
        # ---------------------------------------------------------

        try:

            self.sock.send(frame)

        except Exception as e:

            self.get_logger().error(
                f"CAN transmission failed: {e}"
            )

            return False

        # ---------------------------------------------------------
        # Wait for SDO response
        # ---------------------------------------------------------

        self.sock.settimeout(1.0)

        try:

            while True:

                response = self.sock.recv(16)

                if len(response) < 16:

                    continue

                rx_id, dlc, rx_data = struct.unpack(
                    "=IB3x8s",
                    response
                )

                # -------------------------------------------------
                # Ignore messages from other CAN nodes
                # -------------------------------------------------

                if rx_id != response_id:

                    continue

                # -------------------------------------------------
                # Successful SDO write
                # -------------------------------------------------

                if rx_data[0] == 0x60:

                    self.get_logger().info(
                        f"CAN RX <- "
                        f"Node {node_id} | "
                        f"SDO WRITE SUCCESS"
                    )

                    return True

                # -------------------------------------------------
                # SDO Abort
                # -------------------------------------------------

                if rx_data[0] == 0x80:

                    abort_code = struct.unpack(
                        "<I",
                        rx_data[4:8]
                    )[0]

                    self.get_logger().error(
                        f"Node {node_id}: "
                        f"SDO ABORT "
                        f"0x{abort_code:08X}"
                    )

                    return False

        except socket.timeout:

            self.get_logger().error(
                f"Node {node_id}: "
                f"SDO response timeout."
            )

            return False

        except Exception as e:

            self.get_logger().error(
                f"CAN receive error: {e}"
            )

            return False

    # =============================================================
    # SEND TARGET POSITION
    # =============================================================

    def send_target_position(
        self,
        node_id,
        position_counts
    ):

        # =========================================================
        # STEP 1
        # Write Target Position
        #
        # Object:
        # 0x607A:00
        #
        # Data type:
        # INT32
        #
        # =========================================================

        self.get_logger().info(
            f"Node {node_id}: "
            f"Writing Target Position "
            f"0x607A:00 = {position_counts}"
        )

        success = self.send_sdo_write(
            node_id,
            0x607A,
            0x00,
            position_counts,
            data_size=4
        )

        if not success:

            return False

        # =========================================================
        # STEP 2
        # New Set-Point
        #
        # Controlword:
        # 0x6040:00
        #
        # 0x001F
        #
        # Data type:
        # UINT16
        #
        # =========================================================

        self.get_logger().info(
            f"Node {node_id}: "
            f"New set-point -> "
            f"Controlword 0x001F"
        )

        success = self.send_sdo_write(
            node_id,
            0x6040,
            0x00,
            0x001F,
            data_size=2
        )

        if not success:

            return False

        time.sleep(0.05)

        # =========================================================
        # STEP 3
        # Return to Enable Operation
        #
        # Controlword:
        # 0x000F
        #
        # =========================================================

        self.get_logger().info(
            f"Node {node_id}: "
            f"Return to Enable Operation -> "
            f"Controlword 0x000F"
        )

        success = self.send_sdo_write(
            node_id,
            0x6040,
            0x00,
            0x000F,
            data_size=2
        )

        if not success:

            return False

        self.get_logger().info(
            f"Node {node_id}: "
            f"Target position command complete."
        )

        return True

    # =============================================================
    # SHUTDOWN
    # =============================================================

    def destroy_node(self):

        try:

            self.sock.close()

        except Exception:

            pass

        super().destroy_node()


# =================================================================
# MAIN
# =================================================================

def main(args=None):

    rclpy.init(args=args)

    node = CANopenCommandNode()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        node.get_logger().info(
            "Keyboard interrupt received."
        )

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":

    main()
