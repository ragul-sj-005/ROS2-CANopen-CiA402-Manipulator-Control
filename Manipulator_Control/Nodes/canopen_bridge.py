#!/usr/bin/env python3

import socket
import struct
import time


class CANopenBridge:
    def __init__(self, interface="vcan0", node_id=2):
        self.interface = interface
        self.node_id = node_id

        # SDO request/response CAN IDs
        self.tx_id = 0x600 + node_id
        self.rx_id = 0x580 + node_id

        self.sock = socket.socket(
            socket.AF_CAN,
            socket.SOCK_RAW,
            socket.CAN_RAW
        )

        self.sock.bind((interface,))
        self.sock.settimeout(1.0)

        print(f"CANopen bridge connected to {interface}")
        print(f"Node ID      : {node_id}")
        print(f"SDO TX       : 0x{self.tx_id:03X}")
        print(f"SDO RX       : 0x{self.rx_id:03X}")

    def send_sdo_write(self, index, subindex, data, datatype="int32"):
        """
        Perform an expedited SDO write.

        datatype:
            uint8
            uint16
            uint32
            int8
            int16
            int32
        """

        command_map = {
            "uint8":  0x2F,
            "int8":   0x2F,

            "uint16": 0x2B,
            "int16":  0x2B,

            "uint32": 0x23,
            "int32":  0x23,
        }

        command = command_map[datatype]

        if datatype in ("uint8", "int8"):
            payload = struct.pack("<B", data & 0xFF) + b"\x00\x00\x00"

        elif datatype in ("uint16", "int16"):
            payload = struct.pack("<H", data & 0xFFFF) + b"\x00\x00"

        elif datatype in ("uint32", "int32"):
            payload = struct.pack("<I", data & 0xFFFFFFFF)

        frame_data = struct.pack(
            "<BHB",
            command,
            index,
            subindex
        ) + payload

        can_frame = struct.pack(
            "=IB3x8s",
            self.tx_id,
            8,
            frame_data
        )

        print(
            f"SDO WRITE  "
            f"0x{index:04X}:{subindex:02X} = {data}"
        )

        self.sock.send(can_frame)

        # Wait for SDO response
        try:
            response = self.sock.recv(16)

            can_id, dlc, response_data = struct.unpack(
                "=IB3x8s",
                response
            )

            if can_id != self.rx_id:
                print(
                    f"Unexpected CAN ID: "
                    f"0x{can_id:03X}"
                )
                return False

            # 0x60 = successful SDO download response
            if response_data[0] == 0x60:
                print("SDO WRITE successful")
                return True

            # 0x80 = SDO abort
            if response_data[0] == 0x80:
                abort_code = struct.unpack(
                    "<I",
                    response_data[4:8]
                )[0]

                print(
                    f"SDO ABORT: "
                    f"0x{abort_code:08X}"
                )

                return False

            print(
                "Unexpected SDO response:",
                response_data.hex(" ")
            )

            return False

        except socket.timeout:
            print("ERROR: No SDO response received")
            return False

    def set_operation_mode(self, mode=1):
        """
        CiA402 Modes of Operation

        1 = Profile Position Mode
        """

        return self.send_sdo_write(
            0x6060,
            0x00,
            mode,
            "int8"
        )

    def set_controlword(self, value):
        """
        CiA402 Controlword - 0x6040
        """

        return self.send_sdo_write(
            0x6040,
            0x00,
            value,
            "uint16"
        )

    def set_target_position(self, position):
        """
        CiA402 Target Position - 0x607A

        position must be encoder counts.
        """

        return self.send_sdo_write(
            0x607A,
            0x00,
            position,
            "int32"
        )

    def enable_drive(self):
        """
        CiA402 state sequence:

        Shutdown
        Switch On
        Enable Operation
        """

        print("\nEnabling CiA402 drive...")

        # Shutdown
        if not self.set_controlword(0x0006):
            return False

        time.sleep(0.1)

        # Switch ON
        if not self.set_controlword(0x0007):
            return False

        time.sleep(0.1)

        # Enable operation
        if not self.set_controlword(0x000F):
            return False

        time.sleep(0.1)

        print("Drive enabled.")

        return True

    def move_to_position(self, position):
        """
        Profile Position Mode movement.
        """

        print(
            f"\nMoving to position: {position}"
        )

        # Write target position
        if not self.set_target_position(position):
            return False

        time.sleep(0.05)

        # New set-point
        if not self.set_controlword(0x001F):
            return False

        time.sleep(0.05)

        # Clear new-set-point bit
        if not self.set_controlword(0x000F):
            return False

        print("Target position command sent.")

        return True

    def close(self):
        self.sock.close()


def main():

    bridge = CANopenBridge(
        interface="vcan0",
        node_id=2
    )

    try:

        # Set Profile Position Mode
        if not bridge.set_operation_mode(1):
            print("Failed to set operation mode")
            return

        time.sleep(0.1)

        # Enable CiA402 drive
        if not bridge.enable_drive():
            print("Failed to enable drive")
            return

        # TEST POSITION
        #
        # 1000 encoder counts
        #
        bridge.move_to_position(1000)

    finally:
        bridge.close()


if __name__ == "__main__":
    main()
