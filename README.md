# IPB ROS2 Gazebo Simulation

This project focuses on building a ROS 2-based autonomous navigation system that uses 3D LiDAR for accurate robot localization in prebuilt maps. While most existing systems rely on 2D methods, this work extends localization into full 3D, improving navigation in complex environments.

The system is first developed in simulation (Gazebo) and later deployed on a real robot, ensuring real-time performance using C++.

🎯 Key Goals
Develop a 3D mapping system using LiDAR
Implement 3D localization with scan-to-map registration (ICP)
Handle global initialization without prior pose
Build a complete navigation stack (planner + controller)
Demonstrate autonomous point-to-point navigation

⚙️ Core Idea
The robot uses 3D LiDAR data + prior maps to estimate its position and navigate reliably, even in challenging environments where 2D approaches fail. 

---

## Prerequisites

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic
- Terminator (recommended)

---

## Workspace Setup

Clone the repository:

```bash
git clone git@github.com:noelinnocentlog/3d-lidar-localization.git
cd 3d-lidar-localization
```

Build the workspace:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

Source the workspace:

```bash
source install/setup.bash
```

> 💡 It is recommended to use Terminator since multiple terminals are required.

---

# Running the Navigation Stack

Open a new terminal for each component.

## 1. Start the Simulation

```bash
ros2 launch ipb_ros2_sim all.launch.py sim_config:=indoor
```

---

## 2. Mapping

A map is already provided in this repository.

You only need to run mapping if you want to create a new map.

```bash
ros2 launch opensource_2d_mapping cartographer.launch.py
```

---

## 3. Localization

Localize the robot within the existing map.

```bash
ros2 launch opensource_2d_localization localization.launch.py
```

---

## 4. Planning

Start the planner.

```bash
ros2 launch opensource_2d_planning planning.launch.py
```

---

## 5. Control

Start the controller.

```bash
ros2 launch opensource_2d_control control.launch.py
```

---

## Teleoperation

Control the robot manually using the keyboard:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args \
  -p stamped:=true \
  -r cmd_vel:=/P1_robot0/cmd_vel
```

---

## Troubleshooting

### ROS packages not found

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

### Rebuild workspace

```bash
rm -rf build install log
colcon build --symlink-install
```

### Check ROS topics

```bash
ros2 topic list
```

### Check TF tree

```bash
ros2 run tf2_tools view_frames
```

---

