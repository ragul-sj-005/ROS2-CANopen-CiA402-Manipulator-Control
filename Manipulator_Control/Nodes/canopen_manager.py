
#!/usr/bin/env python3

import socket
import struct
import subprocess
import time

import rclpy
from rclpy.node import Node

from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition


class CANopenManager(Node):

    def __init__(self):
        super().__init__("canopen_manager")

        # =========================================================
        # CANopen configuration
        # =========================================================

        self.interface = "vcan0"

        self.nodes = {
            2: "joint1",
            3: "joint2",
            4: "joint3",
            5: "joint4",
        }

        self.eds_file = (
            "/opt/ros/humble/share/"
            "canopen_fake_slaves/config/cia402_slave.eds"
        )

        # Processes started by THIS manager
        self.slave_processes = {}

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
        # Lifecycle clients
        # =========================================================

        self.lifecycle_clients = {}

        for node_id in self.nodes:

            node_name = f"/cia402_slave_{node_id - 1}"

            self.lifecycle_clients[node_id] = (
                self.create_client(
                    ChangeState,
                    f"{node_name}/change_state"
                )
            )

        # =========================================================
        # Logging
        # =========================================================

        self.get_logger().info(
            "============================================"
        )

        self.get_logger().info(
            "       CANopen 4-Joint Manager"
        )

        self.get_logger().info(
            f"CAN interface : {self.interface}"
        )

        for node_id, joint in self.nodes.items():

            self.get_logger().info(
                f"Node {node_id} -> {joint}"
            )

        self.get_logger().info(
            "============================================"
        )

    # =============================================================
    # Check whether a lifecycle service already exists
    # =============================================================

    def service_available(self, node_id):

        client = self.lifecycle_clients[node_id]

        return client.wait_for_service(
            timeout_sec=0.2
        )

    # =============================================================
    # Start ONE CiA402 slave
    # =============================================================

    def start_slave(self, node_id, joint_name):

        slave_number = node_id - 1

        ros_node_name = (
            f"cia402_slave_{slave_number}"
        )

        # ---------------------------------------------------------
        # If the node is already running, don't start another one.
        # ---------------------------------------------------------

        if self.service_available(node_id):

            self.get_logger().info(
                f"Node {node_id} ({joint_name}) "
                f"is already running."
            )

            return True

        # ---------------------------------------------------------
        # Command used to start fake CiA402 slave
        # ---------------------------------------------------------

        command = [
            "ros2",
            "run",
            "canopen_fake_slaves",
            "cia402_slave_node",

            "--ros-args",

            "-r",
            f"__node:={ros_node_name}",

            "-p",
            f"can_interface_name:={self.interface}",

            "-p",
            f"node_id:={node_id}",

            "-p",
            f"slave_config:={self.eds_file}",
        ]

        self.get_logger().info(
            f"Starting Node {node_id} ({joint_name})..."
        )

        try:

            process = subprocess.Popen(
                command,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )

            self.slave_processes[node_id] = process

        except Exception as e:

            self.get_logger().error(
                f"Could not start Node {node_id}: {e}"
            )

            return False

        # ---------------------------------------------------------
        # Wait for the lifecycle service
        # ---------------------------------------------------------

        self.get_logger().info(
            f"Waiting for Node {node_id} to initialize..."
        )

        client = self.lifecycle_clients[node_id]

        if not client.wait_for_service(
            timeout_sec=10.0
        ):

            self.get_logger().error(
                f"Node {node_id} did not become available."
            )

            return False

        self.get_logger().info(
            f"Node {node_id} ({joint_name}) detected."
        )

        # Small stabilization delay.
        time.sleep(0.2)

        return True

    # =============================================================
    # Start / detect all four slaves
    # =============================================================

    def start_all_slaves(self):

        self.get_logger().info(
            ""
        )

        self.get_logger().info(
            "========== STARTING CiA402 SLAVES =========="
        )

        for node_id, joint_name in self.nodes.items():

            if not self.start_slave(
                node_id,
                joint_name
            ):

                self.get_logger().error(
                    f"Failed to start Node {node_id}."
                )

                return False

            # Small gap between slave startups.
            time.sleep(0.3)

        self.get_logger().info(
            "All four CiA402 nodes are available."
        )

        return True

    # =============================================================
    # Lifecycle transition
    # =============================================================

    def lifecycle_transition(
        self,
        node_id,
        transition_id
    ):

        client = self.lifecycle_clients[node_id]

        request = ChangeState.Request()

        request.transition.id = transition_id

        future = client.call_async(request)

        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=5.0
        )

        if not future.done():

            self.get_logger().error(
                f"Node {node_id}: "
                "lifecycle transition timeout."
            )

            return False

        try:

            result = future.result()

        except Exception as e:

            self.get_logger().error(
                f"Node {node_id}: lifecycle error: {e}"
            )

            return False

        return result.success

    # =============================================================
    # Configure all nodes
    # =============================================================

    def configure_all(self):

        self.get_logger().info(
            ""
        )

        self.get_logger().info(
            "========== CONFIGURING 4 NODES =========="
        )

        for node_id, joint_name in self.nodes.items():

            self.get_logger().info(
                f"Configuring Node {node_id} ({joint_name})..."
            )

            if not self.lifecycle_transition(
                node_id,
                Transition.TRANSITION_CONFIGURE
            ):

                self.get_logger().error(
                    f"Node {node_id} configuration FAILED."
                )

                return False

            self.get_logger().info(
                f"Node {node_id} configured."
            )

        return True

    # =============================================================
    # Activate all nodes
    # =============================================================

    def activate_all(self):

        self.get_logger().info(
            ""
        )

        self.get_logger().info(
            "========== ACTIVATING 4 NODES =========="
        )

        for node_id, joint_name in self.nodes.items():

            self.get_logger().info(
                f"Activating Node {node_id} ({joint_name})..."
            )

            if not self.lifecycle_transition(
                node_id,
                Transition.TRANSITION_ACTIVATE
            ):

                self.get_logger().error(
                    f"Node {node_id} activation FAILED."
                )

                return False

            self.get_logger().info(
                f"Node {node_id} active."
            )

        return True

    # =============================================================
    # Drain stale CAN frames
    # =============================================================

    def drain_can_socket(self):

        self.sock.setblocking(False)

        while True:

            try:
                self.sock.recv(16)

            except BlockingIOError:
                break

            except Exception:
                break

    # =============================================================
    # Send SDO write with retry
    # =============================================================

    def send_sdo_write(
        self,
        node_id,
        index,
        subindex,
        value,
        size=4,
        retries=3
    ):

        request_id = 0x600 + node_id
        response_id = 0x580 + node_id

        # ---------------------------------------------------------
        # Select expedited SDO command
        # ---------------------------------------------------------

        if size == 4:

            command = 0x23

            data = struct.pack(
                "<I",
                value & 0xFFFFFFFF
            )

        elif size == 2:

            command = 0x2B

            data = struct.pack(
                "<H",
                value & 0xFFFF
            )

        elif size == 1:

            command = 0x2F

            data = struct.pack(
                "<B",
                value & 0xFF
            )

        else:

            self.get_logger().error(
                "Invalid SDO size."
            )

            return False

        frame_data = struct.pack(
            "<BHB",
            command,
            index,
            subindex
        ) + data + bytes(4 - size)

        frame = struct.pack(
            "=IB3x8s",
            request_id,
            8,
            frame_data
        )

        # ---------------------------------------------------------
        # Retry loop
        # ---------------------------------------------------------

        for attempt in range(1, retries + 1):

            self.get_logger().info(
                f"Node {node_id}: "
                f"SDO WRITE "
                f"0x{index:04X}:{subindex:02X} = {value} "
                f"(attempt {attempt}/{retries})"
            )

            # Remove stale CAN responses.
            self.drain_can_socket()

            try:

                self.sock.send(frame)

            except Exception as e:

                self.get_logger().error(
                    f"CAN transmission failed: {e}"
                )

                return False

            # -----------------------------------------------------
            # Wait for matching response
            # -----------------------------------------------------

            self.sock.settimeout(1.0)

            start_time = time.time()

            while time.time() - start_time < 1.0:

                try:

                    response = self.sock.recv(16)

                except socket.timeout:

                    break

                except Exception:

                    break

                if len(response) < 16:
                    continue

                rx_id, dlc, rx_data = struct.unpack(
                    "=IB3x8s",
                    response
                )

                # Ignore messages from other nodes.
                if rx_id != response_id:
                    continue

                # -------------------------------------------------
                # Successful SDO download response
                # -------------------------------------------------

                if rx_data[0] == 0x60:

                    self.get_logger().info(
                        f"Node {node_id}: "
                        "SDO WRITE successful"
                    )

                    self.sock.setblocking(False)

                    return True

                # -------------------------------------------------
                # SDO abort
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

                    self.sock.setblocking(False)

                    return False

            self.get_logger().warning(
                f"Node {node_id}: "
                "SDO response timeout"
            )

            time.sleep(0.1)

        self.sock.setblocking(False)

        self.get_logger().error(
            f"Node {node_id}: "
            f"SDO WRITE FAILED "
            f"0x{index:04X}:{subindex:02X}"
        )

        return False

    # =============================================================
    # Enable ONE CiA402 drive
    # =============================================================

    def enable_drive(
        self,
        node_id,
        joint_name
    ):

        self.get_logger().info(
            "--------------------------------------------"
        )

        self.get_logger().info(
            f"Enabling Node {node_id} ({joint_name})"
        )

        # ---------------------------------------------------------
        # 1. Profile Position Mode
        #
        # 0x6060 = Modes of Operation
        # 1      = Profile Position Mode
        # ---------------------------------------------------------

        if not self.send_sdo_write(
            node_id,
            0x6060,
            0x00,
            1,
            size=1
        ):

            return False

        # ---------------------------------------------------------
        # 2. CiA402 state machine
        #
        # 6  = Shutdown
        # 7  = Switch On
        # 15 = Enable Operation
        # ---------------------------------------------------------

        controlwords = [6, 7, 15]

        for controlword in controlwords:

            if not self.send_sdo_write(
                node_id,
                0x6040,
                0x00,
                controlword,
                size=2
            ):

                return False

            time.sleep(0.1)

        self.get_logger().info(
            f"Node {node_id} ({joint_name}) ENABLED."
        )

        return True

    # =============================================================
    # Enable all four drives
    # =============================================================

    def enable_all_drives(self):

        self.get_logger().info(
            ""
        )

        self.get_logger().info(
            "========== ENABLING 4 DRIVES =========="
        )

        for node_id, joint_name in self.nodes.items():

            if not self.enable_drive(
                node_id,
                joint_name
            ):

                self.get_logger().error(
                    f"Node {node_id} enable FAILED."
                )

                return False

        return True

    # =============================================================
    # Complete startup sequence
    # =============================================================

    def start_system(self):

        self.get_logger().info(
            ""
        )

        self.get_logger().info(
            "============================================"
        )

        self.get_logger().info(
            "        CANopen SYSTEM START"
        )

        self.get_logger().info(
            "============================================"
        )

        # ---------------------------------------------------------
        # STEP 1
        # Automatically start / detect slaves
        # ---------------------------------------------------------

        if not self.start_all_slaves():
            return False

        # ---------------------------------------------------------
        # STEP 2
        # Configure
        # ---------------------------------------------------------

        if not self.configure_all():
            return False

        # ---------------------------------------------------------
        # STEP 3
        # Activate
        # ---------------------------------------------------------

        if not self.activate_all():
            return False

        # ---------------------------------------------------------
        # STEP 4
        # Enable drives
        # ---------------------------------------------------------

        if not self.enable_all_drives():
            return False

        # ---------------------------------------------------------
        # READY
        # ---------------------------------------------------------

        self.get_logger().info(
            ""
        )

        self.get_logger().info(
            "============================================"
        )

        self.get_logger().info(
            "        CANopen SYSTEM READY"
        )

        self.get_logger().info(
            "============================================"
        )

        for node_id, joint_name in self.nodes.items():

            self.get_logger().info(
                f"Node {node_id} -> "
                f"{joint_name} -> ENABLED"
            )

        self.get_logger().info(
            "============================================"
        )

        return True

    # =============================================================
    # Shutdown
    # =============================================================

    def shutdown(self):

        self.get_logger().info(
            "Shutting down CANopen manager..."
        )

        # ---------------------------------------------------------
        # Close CAN socket
        # ---------------------------------------------------------

        try:
            self.sock.close()
        except Exception:
            pass

        # ---------------------------------------------------------
        # Stop only processes started by this manager
        # ---------------------------------------------------------

        for node_id, process in self.slave_processes.items():

            try:

                if process.poll() is None:

                    self.get_logger().info(
                        f"Stopping Node {node_id}..."
                    )

                    process.terminate()

                    try:
                        process.wait(timeout=2.0)

                    except subprocess.TimeoutExpired:

                        process.kill()

            except Exception:
                pass

        self.slave_processes.clear()


# =================================================================
# MAIN
# =================================================================

def main(args=None):

    rclpy.init(args=args)

    manager = CANopenManager()

    try:

        success = manager.start_system()

        if not success:

            manager.get_logger().error(
                ""
            )

            manager.get_logger().error(
                "CANopen SYSTEM STARTUP FAILED."
            )

            return

        # ---------------------------------------------------------
        # Keep manager alive.
        #
        # The four CiA402 slave processes remain alive because
        # they were launched by this manager.
        # ---------------------------------------------------------

        rclpy.spin(manager)

    except KeyboardInterrupt:

        manager.get_logger().info(
            "Keyboard interrupt received."
        )

    finally:

        manager.shutdown()

        try:
            manager.destroy_node()
        except Exception:
            pass

        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()

