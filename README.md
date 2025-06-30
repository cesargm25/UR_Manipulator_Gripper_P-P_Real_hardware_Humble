
# UR3e + Robotiq 2F-140 + RealSense Integration on ROS 2 Humble

> ⚠️ This repository is not yet a fully structured ROS 2 package — it is a collection of nodes, scripts, and configuration files meant for **integration and experimentation**.
---

````markdown


This repository contains a **collection of ROS 2 Humble scripts and launch setups** to control a **UR3e robotic arm**, a **Robotiq 2F-140 gripper**, and a **RealSense camera**, intended for use with real hardware.

---

## Dependencies

This setup builds on the following open-source projects:

- **[IFRA-Cranfield ROS 2 Robotiq Gripper Driver](https://github.com/IFRA-Cranfield/IFRA_LinkAttacher)**  
  Used to interface with the Robotiq 2F-140 gripper over Modbus TCP.

- **[Universal Robots ROS 2 Driver (UR Driver)](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)**  
  Official driver for controlling UR3e (and other UR series) robots using ROS 2.

---

## Features

- ✅ **Send joint trajectories** to the UR3e via `/scaled_joint_trajectory_controller`.
- ✅ **Control Robotiq 2F-140** gripper via ROS 2 services.
- ✅ **Open/Close gripper** via Python service clients.
- Integrate RealSense camera for perception tasks.
- Future support for MoveIt 2 and camera-based pick-and-place.

---

## Getting Started

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
````

Then build:

```bash
cd ~/ws_moveit
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

---

##  Running the System

### 1. Start the Robotiq Gripper Server

```bash
ros2 run ros2_robotiqgripper server.py --ros-args -p IPAddress:="192.168.1.102"
```

Or integrate these calls in your Python trajectory script.

### 3. Run Joint Trajectory Publisher

Example in this repository:
`joint_trajectory_publisher.py`

Publishes a trajectory to the robot and automatically calls gripper actions.

##  Structure

```
your_repo/
│
├── joint_trajectory_publisher.py      # Sends joint trajectories and controls the gripper
├── README.md                          # You are here
└── launch/                            # Optional launch files (in development)
```

---

## License

This project uses and extends code from open-source projects under permissive licenses (Apache/MIT/BSD).
Please refer to each original repository for licensing terms.

