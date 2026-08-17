# ROS 2 CANopen-Based Robotic Manipulator Control with Inverse Kinematics and Gazebo

A ROS 2 based robotic manipulator control system integrating **Inverse Kinematics (IK)**, **CANopen communication**, **CiA 402 drive control**, **SocketCAN virtual CAN (`vcan0`)**, and **Gazebo simulation**.

The project demonstrates an end-to-end control architecture in which a desired Cartesian end-effector position is converted into joint angles using inverse kinematics, transmitted through ROS 2 topics, converted into CANopen SDO commands, interpreted as target positions for CiA 402 servo nodes, and finally reflected in the simulated robotic manipulator in Gazebo.

The system also provides Gazebo end-effector feedback to the IK controller, allowing the target position and actual simulated position to be compared.

---

## Table of Contents

* [Project Overview](#project-overview)
* [Project Objectives](#project-objectives)
* [Key Technologies](#key-technologies)
* [System Architecture](#system-architecture)
* [Complete Data Flow](#complete-data-flow)
* [ROS 2 Nodes](#ros-2-nodes)
* [ROS 2 Topics](#ros-2-topics)
* [CANopen Architecture](#canopen-architecture)
* [CANopen Node Mapping](#canopen-node-mapping)
* [CANopen Message IDs](#canopen-message-ids)
* [CiA 402 Drive Control](#cia-402-drive-control)
* [Object Dictionary](#object-dictionary)
* [SDO Communication](#sdo-communication)
* [Target Position Control](#target-position-control)
* [Gazebo Integration](#gazebo-integration)
* [Inverse Kinematics](#inverse-kinematics)
* [Feedback Loop](#feedback-loop)
* [Package Structure](#package-structure)
* [Important File Paths](#important-file-paths)
* [Launch Files](#launch-files)
* [Python Entry Points](#python-entry-points)
* [Installation](#installation)
* [Building the Workspace](#building-the-workspace)
* [Sourcing the Workspace](#sourcing-the-workspace)
* [Starting the System](#starting-the-system)
* [Running the IK Node](#running-the-ik-node)
* [Running the Backend System](#running-the-backend-system)
* [Testing the ROS 2 System](#testing-the-ros-2-system)
* [Inspecting ROS 2 Nodes](#inspecting-ros-2-nodes)
* [Inspecting ROS 2 Topics](#inspecting-ros-2-topics)
* [Using RQT Graph](#using-rqt-graph)
* [Monitoring CANopen Traffic](#monitoring-canopen-traffic)
* [Understanding the Terminal Output](#understanding-the-terminal-output)
* [Complete Control Sequence](#complete-control-sequence)
* [Example Execution](#example-execution)
* [Gazebo Position Feedback](#gazebo-position-feedback)
* [Error Calculation](#error-calculation)
* [Why Virtual CAN is Used](#why-virtual-can-is-used)
* [Real Hardware Adaptation](#real-hardware-adaptation)
* [Troubleshooting](#troubleshooting)
* [Common ROS 2 Commands](#common-ros-2-commands)
* [Engineering Concepts Demonstrated](#engineering-concepts-demonstrated)
* [Future Improvements](#future-improvements)
* [Author](#author)

---

# Project Overview

This project implements a complete robotic manipulator control pipeline using ROS 2.

The system accepts a desired Cartesian position:

```text
X, Y, Z

Example:
X = 0.260 m
Y = 0.100 m
Z = 0.120 m
```

The IK controller calculates the required joint angles.

The resulting joint angles are published through:

```text
/ik/joint_angles
```

The CANopen command node subscribes to this topic and converts the required joint positions into CANopen commands.

The CANopen communication uses:

```text
SocketCAN
    |
    v
  vcan0
```

Four simulated CiA 402 CANopen drives represent four manipulator joints:

```text
joint1 -> CANopen Node 2
joint2 -> CANopen Node 3
joint3 -> CANopen Node 4
joint4 -> CANopen Node 5
```

The target position is transmitted using the CiA 402 target-position object:

```text
0x607A:00
```

The CANopen angle decoder monitors these CANopen target-position frames and converts the received position counts back into joint angles.

Those decoded joint angles are published to:

```text
/decoded_joint_angles
```

The Gazebo joint command node subscribes to that topic and sends the corresponding joint trajectory to the simulated robot.

Gazebo then updates the manipulator configuration.

A Gazebo feedback reader obtains the simulated end-effector position and publishes:

```text
/gazebo/end_effector_position
```

The IK controller subscribes to this feedback and compares the actual position against the target position.

Therefore, the project forms the following complete chain:

```text
Cartesian Target
      |
      v
Inverse Kinematics
      |
      | /ik/joint_angles
      v
CANopen Command Node
      |
      | CANopen SDO
      v
SocketCAN / vcan0
      |
      v
CiA 402 Drive Nodes
      |
      v
Target Position
      |
      v
CANopen Angle Decoder
      |
      | /decoded_joint_angles
      v
Gazebo Joint Command Node
      |
      v
Gazebo Robot
      |
      | /gazebo/end_effector_position
      v
IK Controller
      |
      v
Position Error
```

---

# Project Objectives

The main objectives of this project are:

* Implement inverse kinematics for a robotic manipulator.
* Integrate ROS 2 communication between multiple nodes.
* Control multiple joints using CANopen.
* Implement CiA 402 drive-state control.
* Use SDO communication for drive configuration and target-position commands.
* Simulate four CANopen slave nodes.
* Use SocketCAN and `vcan0` for CAN communication.
* Decode CANopen target-position data.
* Interface the CANopen layer with Gazebo.
* Obtain simulated end-effector feedback from Gazebo.
* Calculate Cartesian position error.
* Visualize the complete ROS 2 communication architecture using RQT Graph.

---

# Key Technologies

## Robotics

* Robotic Manipulator
* Forward Kinematics
* Inverse Kinematics
* Joint-space Control
* Cartesian-space Control

## ROS 2

* ROS 2 Humble
* `rclpy`
* ROS 2 Nodes
* ROS 2 Topics
* Publishers
* Subscribers
* ROS 2 Launch
* ROS 2 Parameters
* RQT Graph

## CANopen

* CANopen
* SocketCAN
* Virtual CAN (`vcan0`)
* SDO
* CAN IDs
* Object Dictionary
* CiA 402
* Controlword
* Mode of Operation
* Target Position

## Simulation

* Gazebo
* ROS 2 Control
* Joint Trajectory Controller
* Simulated CiA 402 slaves

## Programming

* Python
* Linux
* ROS 2 Python Package
* `setuptools`

---

# System Architecture

The system is divided into several logical layers.

```text
+------------------------------------------------------+
|                  USER / OPERATOR                     |
|                                                      |
|              Cartesian Target X, Y, Z               |
+---------------------------+--------------------------+
                            |
                            v
+------------------------------------------------------+
|                 IK CONTROLLER                        |
|                                                      |
|      Cartesian Position -> Joint Angles             |
+---------------------------+--------------------------+
                            |
                            | /ik/joint_angles
                            v
+------------------------------------------------------+
|              CANopen COMMAND NODE                   |
|                                                      |
|      Joint Angles -> CANopen Target Position        |
+---------------------------+--------------------------+
                            |
                            | SDO
                            v
+------------------------------------------------------+
|                    SocketCAN                        |
|                      vcan0                           |
+---------------------------+--------------------------+
                            |
                            v
+------------------------------------------------------+
|                 CiA 402 SLAVES                      |
|                                                      |
| Node 2 -> Joint 1                                    |
| Node 3 -> Joint 2                                    |
| Node 4 -> Joint 3                                    |
| Node 5 -> Joint 4                                    |
+---------------------------+--------------------------+
                            |
                            v
+------------------------------------------------------+
|             CANopen ANGLE DECODER                   |
|                                                      |
|     CANopen Position Counts -> Joint Angles         |
+---------------------------+--------------------------+
                            |
                            | /decoded_joint_angles
                            v
+------------------------------------------------------+
|          GAZEBO JOINT COMMAND NODE                  |
|                                                      |
|        Joint Angles -> Joint Trajectory             |
+---------------------------+--------------------------+
                            |
                            v
+------------------------------------------------------+
|                      GAZEBO                         |
|                                                      |
|              Simulated Manipulator                  |
+---------------------------+--------------------------+
                            |
                            | End-Effector Position
                            v
+------------------------------------------------------+
|             GAZEBO FEEDBACK READER                  |
+---------------------------+--------------------------+
                            |
                            | /gazebo/end_effector_position
                            v
+------------------------------------------------------+
|                 IK CONTROLLER                       |
|                                                      |
|       Target Position - Actual Position = Error     |
+------------------------------------------------------+
```

---

# Complete Data Flow

The complete control process is:

```text
1. User enters X, Y, Z
          |
          v
2. IK controller receives target
          |
          v
3. Inverse kinematics calculates joint angles
          |
          v
4. Joint angles published to /ik/joint_angles
          |
          v
5. CANopen command node receives joint angles
          |
          v
6. Joint angles converted into position counts
          |
          v
7. CANopen SDO writes target position to 0x607A:00
          |
          v
8. CiA 402 slave receives target position
          |
          v
9. CANopen angle decoder monitors the CAN frame
          |
          v
10. Position counts converted back into joint angles
          |
          v
11. Decoded angles published to /decoded_joint_angles
          |
          v
12. Gazebo joint command node receives angles
          |
          v
13. Joint trajectory sent to Gazebo
          |
          v
14. Robot moves in Gazebo
          |
          v
15. Gazebo feedback reader obtains EE position
          |
          v
16. Position published to /gazebo/end_effector_position
          |
          v
17. IK controller calculates position error
```

---

# ROS 2 Nodes

The project contains multiple ROS 2 nodes.

## 1. IK Node

Executable:

```text
ik_node
```

ROS 2 node name:

```text
/ik_node
```

### Purpose

* Accept Cartesian target coordinates.
* Perform inverse kinematics.
* Calculate joint angles.
* Publish calculated joint angles.
* Subscribe to Gazebo end-effector feedback.
* Calculate Cartesian position error.
* Display target and actual position information.

Main output:

```text
/ik/joint_angles
```

Main feedback:

```text
/gazebo/end_effector_position
```

The IK node acts as the high-level controller.

---

## 2. CANopen Manager

Executable:

```text
canopen_manager
```

ROS 2 node:

```text
/canopen_manager
```

### Purpose

* Initialize the CANopen system.
* Start the four simulated CiA 402 slave nodes.
* Wait for all four nodes.
* Configure the nodes.
* Activate the nodes.
* Enable the four drives.
* Configure the operation mode.
* Send the CiA 402 controlword sequence.

CANopen interface:

```text
vcan0
```

Node mapping:

```text
Node 2 -> joint1
Node 3 -> joint2
Node 4 -> joint3
Node 5 -> joint4
```

The manager handles startup and initialization of the CANopen drive system.

---

## 3. CANopen Command Node

Executable:

```text
canopen_command_node
```

ROS 2 node:

```text
/canopen_command_node
```

Subscribed topic:

```text
/ik/joint_angles
```

Purpose:

```text
Joint Angles
      |
      v
Convert to Position Counts
      |
      v
CANopen SDO
      |
      v
0x607A:00 Target Position
```

This node is the bridge between the ROS 2 joint-space command and CANopen.

---

## 4. CANopen Monitor

Executable:

```text
canopen_monitor
```

ROS 2 node:

```text
/canopen_monitor
```

### Purpose

* Monitor CANopen traffic.
* Display SDO requests.
* Display SDO responses.
* Monitor heartbeat frames.
* Decode important CANopen information for debugging.

The monitor is useful during development and debugging.

It is intentionally separated from the main control logic.

---

## 5. CANopen Angle Decoder

Executable:

```text
canopen_angle_decoder
```

ROS 2 node:

```text
/canopen_angle_decoder
```

### Purpose

* Monitor CANopen target-position frames.
* Extract position values.
* Convert CANopen position counts into joint angles.
* Publish decoded joint angles.

It monitors:

```text
Node 2 -> 0x602
Node 3 -> 0x603
Node 4 -> 0x604
Node 5 -> 0x605
```

Object:

```text
0x607A:00
```

Meaning:

```text
Target Position
```

Configured counts per revolution:

```text
100000 counts/revolution
```

Output topic:

```text
/decoded_joint_angles
```

---

## 6. Gazebo Feedback Reader

Executable:

```text
gazebo_feedback_reader
```

ROS 2 node:

```text
/gazebo_feedback_reader
```

### Purpose

Obtain the simulated end-effector position from Gazebo.

Read:

```text
X
Y
Z
```

Publish the position to the IK controller.

Output:

```text
/gazebo/end_effector_position
```

Message type:

```text
geometry_msgs/Point
```

---

## 7. Gazebo Joint Command Node

Executable:

```text
gazebo_joint_command_node
```

ROS 2 node:

```text
/gazebo_joint_command_node
```

Subscribed topic:

```text
/decoded_joint_angles
```

Published command:

```text
/arm_controller/joint_trajectory
```

Purpose:

```text
Decoded Joint Angles
        |
        v
Joint Trajectory
        |
        v
Gazebo ROS 2 Control
        |
        v
Simulated Robot
```

This node is the final bridge from the CANopen simulation layer into the Gazebo robot.

---

# ROS 2 Topics

The main ROS 2 topics used by the project are:

| Topic                              | Message Type                | Publisher                   | Subscriber                  | Purpose                   |
| ---------------------------------- | --------------------------- | --------------------------- | --------------------------- | ------------------------- |
| `/ik/joint_angles`                 | Project joint-angle message | `ik_node`                   | `canopen_command_node`      | IK output                 |
| `/decoded_joint_angles`            | Project joint-angle message | `canopen_angle_decoder`     | `gazebo_joint_command_node` | Decoded CANopen positions |
| `/gazebo/end_effector_position`    | `geometry_msgs/Point`       | `gazebo_feedback_reader`    | `ik_node`                   | Gazebo EE feedback        |
| `/arm_controller/joint_trajectory` | Joint trajectory message    | `gazebo_joint_command_node` | Gazebo controller           | Robot joint command       |
| `/joint_states`                    | `sensor_msgs/JointState`    | Joint State Broadcaster     | ROS/Gazebo nodes            | Joint state information   |

The exact custom message type for the joint-angle topics depends on the message definition used in the package.

---

# ROS 2 Topic Communication

The most important ROS 2 command path is:

```text
/ik/joint_angles
          |
          v
       /ik_node
          |
          v
/canopen_command_node
```

The decoded feedback path is:

```text
CANopen
   |
   v
canopen_angle_decoder
   |
   | publishes
   v
/decoded_joint_angles
   |
   v
gazebo_joint_command_node
```

The Gazebo feedback path is:

```text
Gazebo
   |
   v
gazebo_feedback_reader
   |
   | publishes
   v
/gazebo/end_effector_position
   |
   v
ik_node
```

---

# CANopen Architecture

CANopen is used as the communication layer between the controller and the simulated drives.

The project uses:

```text
SocketCAN
    |
    v
vcan0
```

`vcan0` is a virtual CAN interface.

It behaves similarly to a CAN network from the application perspective, but does not require physical CAN hardware.

---

# CANopen Node Mapping

The manipulator contains four controlled joints.

They are mapped to four CANopen node IDs:

| Joint  | CANopen Node ID |
| ------ | --------------: |
| joint1 |               2 |
| joint2 |               3 |
| joint3 |               4 |
| joint4 |               5 |

Therefore:

```text
joint1 -> Node 2
joint2 -> Node 3
joint3 -> Node 4
joint4 -> Node 5
```

This mapping is used throughout the CANopen manager, command node, monitor, and angle decoder.

---

# CANopen Message IDs

For CANopen SDO communication, the standard server/client COB-ID relationship is used.

For node ID `N`:

```text
SDO Request  = 0x600 + N
SDO Response = 0x580 + N
```

Therefore:

| Node | Joint  | SDO Request | SDO Response |
| ---: | ------ | ----------: | -----------: |
|    2 | joint1 |     `0x602` |      `0x582` |
|    3 | joint2 |     `0x603` |      `0x583` |
|    4 | joint3 |     `0x604` |      `0x584` |
|    5 | joint4 |     `0x605` |      `0x585` |

Heartbeat IDs:

```text
0x700 + Node ID
```

Therefore:

```text
Node 2 -> 0x702
Node 3 -> 0x703
Node 4 -> 0x704
Node 5 -> 0x705
```

---

# CiA 402 Drive Control

CiA 402 is the CANopen device profile commonly used for motion-control drives.

The project uses the CiA 402 state machine to configure and enable the simulated drives.

Important objects include:

```text
0x6040:00 -> Controlword
0x6060:00 -> Modes of Operation
0x607A:00 -> Target Position
```

---

## Mode of Operation

The project configures:

```text
0x6060:00 = 1
```

This corresponds to:

```text
Profile Position Mode
```

Therefore, the drive is configured for position-based motion commands.

---

## Controlword

The CiA 402 controlword is:

```text
0x6040:00
```

The drive is enabled using the sequence:

```text
0x0006
   |
   v
Shutdown
   |
   v
0x0007
   |
   v
Switch On
   |
   v
0x000F
   |
   v
Operation Enabled
```

This allows the drive to accept motion commands.

---

# Object Dictionary

CANopen devices expose parameters through an Object Dictionary.

An object is addressed using:

```text
Index : Sub-index
```

For example:

```text
0x607A:00
```

means:

```text
Index    = 0x607A
Subindex = 00
```

Important objects in this project are:

```text
0x6060:00
Modes of Operation

0x6040:00
Controlword

0x607A:00
Target Position
```

---

# SDO Communication

SDO stands for:

```text
Service Data Object
```

SDO is used for client-server communication with the CANopen Object Dictionary.

The project uses SDO expedited writes.

For a 1-byte value:

```text
0x2F
```

For a 2-byte value:

```text
0x2B
```

For a 4-byte value:

```text
0x23
```

The SDO request is sent to:

```text
0x600 + Node ID
```

The response is expected from:

```text
0x580 + Node ID
```

A successful SDO write is acknowledged using:

```text
0x60
```

An SDO abort is indicated by:

```text
0x80
```

---

# Example SDO Communication

For Node 2:

```text
Node ID = 2
```

Request CAN ID:

```text
0x600 + 2 = 0x602
```

Response CAN ID:

```text
0x580 + 2 = 0x582
```

To configure Profile Position Mode:

```text
Object = 0x6060:00
Value  = 1
```

The command node sends an SDO write through:

```text
0x602
```

The corresponding slave responds through:

```text
0x582
```

---

# Target Position Control

The target position object is:

```text
0x607A:00
```

The control flow is:

```text
Joint Angle
     |
     v
Position Conversion
     |
     v
Position Counts
     |
     v
SDO Write
     |
     v
0x607A:00
```

The project uses:

```text
100000 counts/revolution
```

Therefore:

```text
counts = angle / (2π) × counts_per_revolution
```

and:

```text
angle = counts / counts_per_revolution × 2π
```

where:

```text
counts_per_revolution = 100000
```

The exact sign and joint-specific conversion depend on the joint convention implemented in the project.

---

# CANopen Angle Decoder

The angle decoder monitors the SDO traffic associated with:

```text
0x607A:00
```

It receives position counts and converts them into joint angles.

The mapping is:

```text
0x602 -> joint1
0x603 -> joint2
0x604 -> joint3
0x605 -> joint4
```

The resulting angles are then published to:

```text
/decoded_joint_angles
```

This creates an explicit bridge:

```text
CANopen Position Counts
          |
          v
CANopen Angle Decoder
          |
          v
Joint Angles
          |
          v
ROS 2 Topic
```

---

# Gazebo Integration

Gazebo is used to simulate the robotic manipulator.

The Gazebo side contains the simulated robot and ROS 2 control infrastructure.

The joint command path is:

```text
/decoded_joint_angles
          |
          v
gazebo_joint_command_node
          |
          v
/arm_controller/joint_trajectory
          |
          v
arm_controller
          |
          v
Gazebo
          |
          v
Robot Motion
```

The project therefore connects the CANopen control layer to the simulated robot.

---

# Inverse Kinematics

Inverse Kinematics calculates the joint configuration required to place the end effector at a desired Cartesian position.

The input is:

```text
X
Y
Z
```

The output is:

```text
θ1
θ2
θ3
θ4
```

Conceptually:

```text
[X, Y, Z]
     |
     v
Inverse Kinematics
     |
     v
[θ1, θ2, θ3, θ4]
```

These joint angles are then published to:

```text
/ik/joint_angles
```

---

# Feedback Loop

The project does not stop at sending commands.

It also reads the actual simulated end-effector position from Gazebo.

The feedback loop is:

```text
Target Position
      |
      v
IK Controller
      |
      v
Joint Angles
      |
      v
CANopen
      |
      v
Gazebo
      |
      v
Actual EE Position
      |
      v
IK Controller
      |
      v
Position Error
```

This allows the system to evaluate how close the robot is to the target position.

---

# Position Error

For target position:

```text
P_target = [X_target, Y_target, Z_target]
```

and actual position:

```text
P_actual = [X_actual, Y_actual, Z_actual]
```

the Cartesian error is:

```text
e = P_target - P_actual
```

Therefore:

```text
e_x = X_target - X_actual

e_y = Y_target - Y_actual

e_z = Z_target - Z_actual
```

The total Euclidean position error is:

```text
E = sqrt(e_x² + e_y² + e_z²)
```

For example, if:

```text
Target:

X = 0.260000 m
Y = 0.100000 m
Z = 0.120000 m
```

and:

```text
Actual:

X = 0.212990 m
Y = 0.081923 m
Z = 0.195127 m
```

then:

```text
X error =  0.047010 m
Y error =  0.018077 m
Z error = -0.075127 m
```

and:

```text
Total error ≈ 0.090448 m
```

The IK controller can report:

```text
STATUS: POSITION ERROR
```

until the actual position approaches the target.

---

# Package Structure

The ROS 2 package is:

```text
manipulator_ik
```

A typical project structure is:

```text
turtlebot3_ws/
│
├── src/
│   │
│   └── manipulator_ik/
│       │
│       ├── manipulator_ik/
│       │   ├── __init__.py
│       │   ├── ik_node.py
│       │   ├── canopen_manager.py
│       │   ├── canopen_command_node.py
│       │   ├── canopen_monitor.py
│       │   ├── canopen_position_reader.py
│       │   ├── canopen_angle_decoder.py
│       │   ├── gazebo_joint_command_node.py
│       │   └── gazebo_feedback_reader.py
│       │
│       ├── launch/
│       │   ├── ik_only.launch.py
│       │   └── <backend_launch_file>.launch.py
│       │
│       ├── resource/
│       │   └── manipulator_ik
│       │
│       ├── package.xml
│       ├── setup.py
│       └── setup.cfg
│
└── install/
```

Replace:

```text
<backend_launch_file>.launch.py
```

with the actual backend launch filename used in the repository.

---

# Important File Paths

The workspace used during development is:

```text
~/turtlebot3_ws
```

The ROS 2 package is located under:

```text
~/turtlebot3_ws/src/manipulator_ik
```

Python package:

```text
~/turtlebot3_ws/src/manipulator_ik/manipulator_ik
```

Launch directory:

```text
~/turtlebot3_ws/src/manipulator_ik/launch
```

---

# CANopen Manager Configuration

The CANopen manager uses:

```text
vcan0
```

The four nodes are mapped as:

```python
{
    2: "joint1",
    3: "joint2",
    4: "joint3",
    5: "joint4"
}
```

The CiA 402 EDS configuration used during development is:

```text
/opt/ros/humble/share/canopen_fake_slaves/config/cia402_slave.eds
```

This EDS describes the simulated CiA 402 slave device configuration.

---

# Launch Files

The project uses separate launch files to make the system easier to operate.

## IK-only Launch

The IK-only launch file is:

```text
ik_only.launch.py
```

It starts only:

```text
ik_node
```

This is useful when the user wants to interact directly with the IK controller without displaying all backend node output.

Run:

```bash
ros2 launch manipulator_ik ik_only.launch.py
```

The IK node is the frontend interface.

It is responsible for accepting Cartesian input and displaying IK-related output.

---

# Backend Launch

The second launch file starts the backend nodes.

The backend contains the CANopen, Gazebo integration, decoding, and command nodes.

Conceptually:

```text
canopen_manager
canopen_monitor
canopen_angle_decoder
gazebo_feedback_reader
canopen_command_node
gazebo_joint_command_node
```

The backend launch file should be used together with the Gazebo simulation.

---

# Why Separate the Launch Files?

Separating the frontend and backend provides cleaner terminal interaction.

The architecture becomes:

```text
Terminal 1
-------------------------
IK Launch
     |
     v
ik_node
-------------------------


Terminal 2
-------------------------
Backend Launch
     |
     +--> CANopen Manager
     +--> CANopen Monitor
     +--> CANopen Angle Decoder
     +--> CANopen Command Node
     +--> Gazebo Feedback Reader
     +--> Gazebo Joint Command Node
-------------------------
```

This prevents backend diagnostic messages from overwhelming the terminal used for IK interaction.

---

# Python Entry Points

The package uses ROS 2 Python console scripts.

The relevant executables are:

```text
ik_node
canopen_manager
canopen_command_node
canopen_monitor
canopen_position_reader
canopen_angle_decoder
gazebo_joint_command_node
gazebo_feedback_reader
```

They are registered in:

```text
setup.py
```

using:

```python
entry_points={
    'console_scripts': [
        ...
    ],
}
```

This allows ROS 2 to launch them by executable name rather than directly executing the Python files.

---

# Installation

Clone the repository into a ROS 2 workspace:

```bash
cd ~/turtlebot3_ws/src

git clone <YOUR_GITHUB_REPOSITORY_URL>
```

Install required ROS 2 dependencies according to the environment used by the project.

Then return to the workspace:

```bash
cd ~/turtlebot3_ws
```

---

# Building the Workspace

Build only this package:

```bash
colcon build --packages-select manipulator_ik
```

If the package builds successfully:

```text
Starting >>> manipulator_ik
Finished <<< manipulator_ik
```

then source the generated environment:

```bash
source install/setup.bash
```

---

# Important: Launch Files Must Be Installed

The `setup.py` file must include the launch files inside `data_files`.

For example:

```python
data_files=[
    (
        'share/ament_index/resource_index/packages',
        ['resource/' + package_name]
    ),
    (
        'share/' + package_name,
        ['package.xml']
    ),
    (
        'share/' + package_name + '/launch',
        [
            'launch/ik_only.launch.py',
            'launch/<backend_launch_file>.launch.py'
        ]
    ),
]
```

The filenames must exactly match the files that actually exist inside:

```text
launch/
```

For example:

```text
launch/ik_only.launch.py
```

must actually exist.

If `setup.py` contains:

```text
launch/manipulator_system.launch.py
```

but that file does not exist, `colcon build` will fail with:

```text
error: can't copy 'launch/manipulator_system.launch.py':
doesn't exist or not a regular file
```

Therefore, whenever a launch file is renamed or deleted, update `setup.py`.

---

# Sourcing the Workspace

After building:

```bash
source install/setup.bash
```

It is recommended to source the workspace in every terminal used for the project.

For convenience:

```bash
source ~/turtlebot3_ws/install/setup.bash
```

can be added to the shell configuration if appropriate.

---

# Starting the System

A typical execution consists of:

```text
1. Start Gazebo
2. Start the backend nodes
3. Start the IK node
4. Enter Cartesian target
5. Observe robot motion
6. Observe feedback
7. Inspect ROS 2 graph if required
```

The exact Gazebo launch command depends on the robot simulation configuration used with the project.

---

# Running the IK Node

Run:

```bash
ros2 launch manipulator_ik ik_only.launch.py
```

The terminal should display the IK controller.

Example startup:

```text
============================================
          IK CONTROLLER STARTED
============================================

IK output topic:
/ik/joint_angles

Gazebo feedback topic:
/gazebo/end_effector_position

Feedback type:
geometry_msgs/Point
```

The node then waits for user input.

This waiting behavior is expected.

The IK node is interactive because it needs Cartesian target coordinates.

---

# Running the Backend

Start the backend launch file from another terminal.

The backend should provide:

```text
CANopen
+
CiA 402
+
Gazebo command
+
Gazebo feedback
+
CANopen decoding
```

The backend is intended to run continuously while the IK controller is being used.

---

# Testing the ROS 2 System

After starting the system, check the active nodes:

```bash
ros2 node list
```

You should see nodes such as:

```text
/ik_node
/canopen_manager
/canopen_monitor
/canopen_angle_decoder
/gazebo_feedback_reader
/canopen_command_node
/gazebo_joint_command_node
```

Additional Gazebo and ROS 2 Control nodes may also appear.

---

# Inspecting ROS 2 Nodes

To inspect a node:

```bash
ros2 node info /ik_node
```

For the CANopen manager:

```bash
ros2 node info /canopen_manager
```

For the command node:

```bash
ros2 node info /canopen_command_node
```

For the decoder:

```bash
ros2 node info /canopen_angle_decoder
```

For the Gazebo feedback reader:

```bash
ros2 node info /gazebo_feedback_reader
```

For the Gazebo joint command node:

```bash
ros2 node info /gazebo_joint_command_node
```

These commands show:

* Publishers
* Subscribers
* Services
* Actions
* Parameters

---

# Inspecting ROS 2 Topics

List all topics:

```bash
ros2 topic list
```

Inspect a topic:

```bash
ros2 topic info /ik/joint_angles
```

Inspect the Gazebo feedback topic:

```bash
ros2 topic info /gazebo/end_effector_position
```

Inspect decoded joint angles:

```bash
ros2 topic info /decoded_joint_angles
```

Read a topic:

```bash
ros2 topic echo /gazebo/end_effector_position
```

Read IK output:

```bash
ros2 topic echo /ik/joint_angles
```

Read decoded angles:

```bash
ros2 topic echo /decoded_joint_angles
```

Check publishing frequency:

```bash
ros2 topic hz /gazebo/end_effector_position
```

---

# Using RQT Graph

The complete ROS 2 communication graph can be visualized using:

```bash
rqt_graph
```

The graph shows:

```text
Nodes
+
Topics
+
Publishers
+
Subscribers
```

A simplified representation of the important graph is:

```text
                    /ik/joint_angles
                         |
                         v
                    /ik_node
                         |
                         v
              /canopen_command_node
                         |
                         |
                    CANopen
                         |
                         v
                  CiA 402 Slaves
                         |
                         v
              /canopen_angle_decoder
                         |
                         v
               /decoded_joint_angles
                         |
                         v
             /gazebo_joint_command_node
                         |
                         v
                       Gazebo
                         |
                         v
          /gazebo/end_effector_position
                         |
                         v
              /gazebo_feedback_reader
                         |
                         v
                      /ik_node
```

The RQT graph may also display nodes belonging to:

* Gazebo
* ROS 2 Control
* Joint State Broadcaster
* Robot State Publisher
* Controller Manager
* CiA 402 slave nodes

These additional nodes are expected because they are part of the simulation and control infrastructure.

---

# Why Some Nodes Appear Without Topic Connections

In RQT Graph, a node may appear as an isolated node if it communicates through mechanisms that are not represented as ordinary ROS 2 topic edges.

For example:

```text
canopen_manager
```

may primarily perform initialization and CANopen management.

The actual CANopen communication occurs through:

```text
vcan0
```

which is not a ROS 2 topic.

Therefore, RQT Graph should not be interpreted as a complete representation of the CAN bus communication.

The ROS graph represents ROS 2 communication.

The CANopen monitor represents CAN communication.

Both graphs together describe the complete system.

---

# Monitoring CANopen Traffic

The project uses:

```text
vcan0
```

The CAN interface can be inspected using Linux SocketCAN tools.

Check the interface:

```bash
ip link show vcan0
```

Bring it up if necessary:

```bash
sudo ip link set vcan0 up
```

CAN frames can be monitored using:

```bash
candump vcan0
```

This allows low-level CAN traffic to be observed.

Example:

```text
602 ...
603 ...
604 ...
605 ...
```

These correspond to the SDO request IDs for Nodes 2–5.

---

# CANopen Monitoring Architecture

The CANopen monitor observes:

```text
vcan0
```

and interprets important frames.

The project monitors:

```text
SDO Requests:
0x602 - 0x605

SDO Responses:
0x582 - 0x585

Heartbeats:
0x702 - 0x705
```

This provides a useful debugging layer between the application and simulated drives.

---

# Understanding the Terminal Output

The project intentionally separates frontend and backend execution.

The IK terminal is used for:

```text
User input
+
IK results
+
Gazebo feedback
+
Position error
```

Backend nodes can be inspected independently using:

```bash
ros2 node info
```

and:

```bash
ros2 topic echo
```

This provides a cleaner operator interface.

---

# Complete Control Sequence

The complete startup and control sequence is:

```text
START
 |
 v
Start Gazebo
 |
 v
Start CANopen Manager
 |
 v
Create/Start CiA 402 slaves
 |
 v
Wait for Node 2
 |
 v
Wait for Node 3
 |
 v
Wait for Node 4
 |
 v
Wait for Node 5
 |
 v
Configure Nodes
 |
 v
Activate Nodes
 |
 v
Set Mode of Operation
 |
 v
Controlword = 0x0006
 |
 v
Controlword = 0x0007
 |
 v
Controlword = 0x000F
 |
 v
Drives Enabled
 |
 v
Start IK Controller
 |
 v
User enters X,Y,Z
 |
 v
Calculate IK
 |
 v
Publish /ik/joint_angles
 |
 v
CANopen Command Node
 |
 v
Convert angle -> counts
 |
 v
Write 0x607A:00
 |
 v
CiA 402 Slave
 |
 v
CANopen Angle Decoder
 |
 v
Publish /decoded_joint_angles
 |
 v
Gazebo Joint Command Node
 |
 v
Robot Moves
 |
 v
Gazebo Feedback Reader
 |
 v
Publish /gazebo/end_effector_position
 |
 v
IK Controller
 |
 v
Calculate Position Error
 |
 v
END / NEXT TARGET
```

---

# Example Execution

An example Cartesian target can be entered as:

```text
X = 0.260 m
Y = 0.100 m
Z = 0.120 m
```

The IK controller calculates:

```text
θ1
θ2
θ3
θ4
```

These values are published:

```text
/ik/joint_angles
```

The CANopen command node converts each joint angle into position counts.

For:

```text
100000 counts/revolution
```

the conversion is:

```text
counts = angle / (2π) × 100000
```

The target counts are then written to:

```text
0x607A:00
```

The CANopen decoder observes the corresponding CANopen communication and converts the counts back into angles.

The decoded angles are published:

```text
/decoded_joint_angles
```

The Gazebo joint command node converts them into a joint trajectory and sends the trajectory to the simulated manipulator.

Gazebo updates the robot configuration.

The Gazebo feedback reader obtains the actual end-effector position:

```text
X_actual
Y_actual
Z_actual
```

The IK controller compares this against:

```text
X_target
Y_target
Z_target
```

and calculates the position error.

---

# Gazebo Position Feedback

The Gazebo feedback reader publishes:

```text
/gazebo/end_effector_position
```

with message type:

```text
geometry_msgs/Point
```

The message contains:

```text
x
y
z
```

Example:

```text
x: 0.212990
y: 0.081923
z: 0.195127
```

The IK controller uses these values to determine how closely the simulated manipulator reached the requested Cartesian position.

---

# Error Calculation

The Cartesian position error is:

```text
e = P_target - P_actual
```

Component-wise:

```text
e_x = X_target - X_actual
e_y = Y_target - Y_actual
e_z = Z_target - Z_actual
```

The Euclidean error is:

```text
E = sqrt(e_x² + e_y² + e_z²)
```

This gives a single scalar value representing the distance between the desired and actual end-effector positions.

---

# Why Virtual CAN is Used

The project uses:

```text
vcan0
```

instead of physical CAN hardware.

The main advantages are:

* No physical CAN interface required.
* No physical servo drives required.
* Easy development and debugging.
* CAN frames can be monitored using `candump`.
* The complete CANopen communication path can be tested in software.
* The same CANopen concepts can later be transferred to physical hardware.

The architecture therefore provides a hardware-independent development environment.

---

# Real Hardware Adaptation

The current project uses:

```text
vcan0
```

and simulated CiA 402 slaves.

For a physical deployment, the architecture can be adapted by replacing:

```text
vcan0
```

with a physical CAN interface.

Conceptually:

```text
ROS 2
  |
  v
CANopen Command Node
  |
  v
SocketCAN
  |
  v
Physical CAN Interface
  |
  v
CAN Bus
  |
  +--> Servo Drive Node 2
  +--> Servo Drive Node 3
  +--> Servo Drive Node 4
  +--> Servo Drive Node 5
```

The higher-level ROS 2 architecture can remain largely unchanged.

The physical drive configuration would need to match:

* Node IDs
* EDS configuration
* Operation mode
* Encoder scaling
* Position limits
* Controlword sequence
* Statusword handling
* PDO/SDO configuration
* Joint direction conventions

---

# Troubleshooting

## `vcan0` Does Not Exist

Check:

```bash
ip link show
```

If necessary, create a virtual CAN interface:

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

Verify:

```bash
ip link show vcan0
```

---

## No CAN Frames Appear

Run:

```bash
candump vcan0
```

Then start the CANopen backend.

Check whether frames such as:

```text
602
603
604
605
```

appear.

---

## ROS 2 Node Not Found

Check:

```bash
ros2 node list
```

If the node does not appear, verify that the workspace has been sourced:

```bash
source ~/turtlebot3_ws/install/setup.bash
```

Then rebuild:

```bash
cd ~/turtlebot3_ws
colcon build --packages-select manipulator_ik
source install/setup.bash
```

---

## Launch File Not Found

If:

```bash
ros2 launch manipulator_ik ik_only.launch.py
```

fails, verify that the launch file exists:

```bash
ls ~/turtlebot3_ws/src/manipulator_ik/launch/
```

Then verify that the file is included in `setup.py`.

Rebuild:

```bash
colcon build --packages-select manipulator_ik
```

and source:

```bash
source install/setup.bash
```

---

## Gazebo Robot Does Not Move

Check:

```bash
ros2 topic echo /decoded_joint_angles
```

If no data appears, inspect:

```text
IK
  |
  v
/ik/joint_angles
  |
  v
CANopen Command Node
  |
  v
vcan0
  |
  v
CANopen Decoder
  |
  v
/decoded_joint_angles
```

Also verify the Gazebo controller:

```bash
ros2 control list_controllers
```

---

## IK Output Is Correct but Gazebo Position Is Wrong

Check:

```bash
ros2 topic echo /gazebo/end_effector_position
```

Then verify:

* Joint order.
* Joint direction.
* Joint zero offsets.
* Position-count scaling.
* IK coordinate convention.
* Gazebo joint limits.
* Robot model configuration.

---

# Common ROS 2 Commands

## List Nodes

```bash
ros2 node list
```

## List Topics

```bash
ros2 topic list
```

## Inspect Node

```bash
ros2 node info /ik_node
```

## Inspect Topic

```bash
ros2 topic info /ik/joint_angles
```

## Echo Topic

```bash
ros2 topic echo /ik/joint_angles
```

## Check Topic Frequency

```bash
ros2 topic hz /gazebo/end_effector_position
```

## Visualize ROS Graph

```bash
rqt_graph
```

## Check Controllers

```bash
ros2 control list_controllers
```

## Monitor CAN

```bash
candump vcan0
```

## Check CAN Interface

```bash
ip link show vcan0
```

---

# Project Workflow

The overall project workflow can be summarized as:

```text
              USER
                |
                | Cartesian Target
                v
       +------------------+
       |  Inverse         |
       |  Kinematics      |
       +--------+---------+
                |
                | Joint Angles
                v
       +------------------+
       | ROS 2 Topic      |
       | /ik/joint_angles |
       +--------+---------+
                |
                v
       +------------------+
       | CANopen Command  |
       | Node             |
       +--------+---------+
                |
                | SDO
                v
       +------------------+
       | SocketCAN        |
       | vcan0            |
       +--------+---------+
                |
                v
       +------------------+
       | CiA 402 Slaves   |
       | Node 2 - 5       |
       +--------+---------+
                |
                v
       +------------------+
       | CANopen Angle    |
       | Decoder          |
       +--------+---------+
                |
                | Joint Angles
                v
       +------------------+
       | Gazebo Joint     |
       | Command Node     |
       +--------+---------+
                |
                v
       +------------------+
       | Gazebo           |
       | Manipulator      |
       +--------+---------+
                |
                | EE Position
                v
       +------------------+
       | Gazebo Feedback  |
       | Reader           |
       +--------+---------+
                |
                v
       +------------------+
       | IK Controller    |
       | Error Calculation|
       +------------------+
                |
                v
             Feedback
```

---

# Engineering Concepts Demonstrated

This project demonstrates practical integration of several robotics and automation concepts:

### Robotics

* Inverse Kinematics
* Forward Kinematics
* Cartesian-space control
* Joint-space control
* Manipulator control
* End-effector positioning

### ROS 2

* Node-based architecture
* Publisher/subscriber communication
* ROS 2 topics
* Launch files
* Python ROS 2 nodes
* ROS 2 Control
* RQT Graph
* Simulation integration

### CANopen

* CANopen node architecture
* Object Dictionary
* SDO communication
* CAN identifiers
* Heartbeat communication
* CANopen client/server communication
* SocketCAN
* Virtual CAN

### CiA 402

* Motion-control device profile
* Controlword
* Operation mode
* Profile Position Mode
* Target Position
* Drive state machine
* Drive enabling sequence

### Simulation

* Gazebo
* Simulated servo drives
* Virtual CAN
* Closed-loop position feedback
* ROS 2 and Gazebo integration

### Software Engineering

* Modular ROS 2 nodes
* Separation of frontend and backend
* Debugging and monitoring tools
* Linux networking
* Python package structure
* Reproducible development environment

---

# Future Improvements

Possible future extensions include:

* Replace `vcan0` with physical CAN hardware.
* Integrate physical CiA 402 servo drives.
* Add Statusword (`0x6041`) monitoring.
* Implement fault detection and recovery.
* Add PDO-based cyclic communication.
* Implement trajectory interpolation.
* Add velocity and acceleration control.
* Add joint-limit checking.
* Add collision detection.
* Add real-time control.
* Add controller tuning.
* Add automatic Cartesian trajectory generation.
* Add MoveIt 2 integration.
* Add RViz visualization.
* Add automated testing.
* Add hardware-in-the-loop validation.
* Add performance benchmarking.
* Add ROS 2 parameters for configurable CANopen node IDs and scaling.
* Add dynamic encoder scaling for individual joints.

---

# Author

**Ragul S J**

Robotics and Automation Engineering

PSG College of Technology

ROS 2 | Robotics | Industrial Automation | CANopen | CiA 402 | Gazebo | Inverse Kinematics

---

# License

This project is intended for educational, research, and robotics development purposes.

Add the appropriate license file to the repository if the project is distributed publicly.
