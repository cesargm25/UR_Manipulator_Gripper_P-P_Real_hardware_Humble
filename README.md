#  UR3e + Robotiq 2F-140 + RealSense Integration on ROS 2 Humble

This repository contains a **collection of ROS 2 Humble scripts and launch setups** to control a **UR3e robotic arm**, a **Robotiq 2F-140 gripper**, and a **RealSense camera**, intended for use with real hardware.

> ⚠️ This repository is not yet a fully structured ROS 2 package — it is a collection of nodes, scripts, and configuration files meant for **integration and experimentation**.

---

##  Dependencies

This setup builds on the following open-source projects:

- **[IFRA-Cranfield ROS 2 Robotiq Gripper Driver](https://github.com/IFRA-Cranfield/IFRA_LinkAttacher)**  
  Used to interface with the Robotiq 2F-140 gripper over Modbus TCP.

- **[Universal Robots ROS 2 Driver (UR Driver)](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)**  
  Official driver for controlling UR3e (and other UR series) robots using ROS 2.

- **[RealSense ROS 2 Wrapper (Intel)](https://github.com/IntelRealSense/realsense-ros)**  
  (Optional) For integrating Intel RealSense cameras (e.g., D415 or D435).

---

## Features

- ✅ **Send joint trajectories** to the UR3e via `/scaled_joint_trajectory_controller`.
- ✅ **Control Robotiq 2F-140** gripper via ROS 2 services.
- ✅ **Open/Close gripper** via Python service clients.
- ✅ **Integrate RealSense camera** for perception tasks.
-  Future support for MoveIt 2 and camera-based pick-and-place.

---

## 🚀 Getting Started

### 1. Clone and Setup Workspaces

Clone the required dependencies into your ROS 2 workspace (`ws_moveit` or similar):

```bash
cd ~/ws_moveit/src

# Robotiq Gripper Driver
git clone https://github.com/IFRA-Cranfield/ros2_RobotiqGripper -b humble

# UR Driver
git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git

# (Optional) RealSense ROS 2 Wrapper
git clone https://github.com/IntelRealSense/realsense-ros.git
```

# Running the System
1. Start the Robotiq Gripper Server

```bash
ros2 run ros2_robotiqgripper server.py --ros-args -p IPAddress:="192.168.1.102"
```

2. Control the Gripper from a Python Node
You can call:

```bash
ros2 service call /Robotiq_Gripper ros2_robotiqgripper/srv/RobotiqGripper "{action: 'CLOSE'}"
ros2 service call /Robotiq_Gripper ros2_robotiqgripper/srv/RobotiqGripper "{action: 'OPEN'}"
```
Or integrate these calls in your Python trajectory script.

# Run Joint Trajectory Publisher
Example in this repository:

joint_trajectory_publisher.py

Publishes a trajectory to the robot and automatically calls gripper actions.

# RealSense Integration
Make sure the RealSense udev rules are installed.
Launch the camera node:

```bash
ros2 launch realsense2_camera rs_launch.py
Topics like /camera/color/image_raw will be available for perception modules.
```

Structure
UR_Manipulator_Gripper_P-P_Real_hardware_Humble/
│
├── joint_trajectory_publisher.py      # Sends joint trajectories and controls the gripper
├── README.md                          # You are here
└── launch/                            # Optional launch files (in development)


