# 🚀 Robot Navigation using 3D LiDAR Maps

## 📌 Overview

This project focuses on designing and implementing a **3D LiDAR-based autonomous navigation system** using ROS 2. The system enables a robot to localize itself in a prebuilt 3D map and navigate from a start point to a goal position autonomously.

Unlike traditional 2D navigation systems, this project extends localization and mapping into **3D space**, improving performance in complex real-world environments.

---

## 🎯 Objectives

* Develop a full **ROS 2-based navigation pipeline**
* Implement **3D LiDAR mapping**
* Build a **3D localization system** using scan-to-map registration
* Enable **autonomous navigation (A → B)**
* Demonstrate system performance in **simulation and real-world deployment**

---

## 🧠 System Architecture

```
3D LiDAR → Mapping → Localization → Planning → Control → Robot Motion
```

---

## 🛠️ Technologies Used

* ROS 2 (Robot Operating System)
* Gazebo (Simulation Environment)
* C++ (Core Implementation)
* 3D LiDAR Sensors (e.g., RoboSense, Hesai)
* ICP (Iterative Closest Point)
* SLAM & Localization Algorithms

---

## 🔧 Project Modules

### 1. 2D Navigation Stack

* Visual marker-based pose estimation
* 2D occupancy grid mapping
* Path planning & control
* Baseline system for understanding navigation

---

### 2. 3D Mapping System

* Generate 3D occupancy maps using LiDAR
* Filter dynamic objects
* Produce high-quality maps for localization

---

### 3. Scan-to-Map Registration

* ICP-based alignment
* Real-time pose estimation
* Efficient data association techniques

---

### 4. Global Localization

* Estimate initial robot pose without prior knowledge
* Implement global registration techniques
* Refine pose using geometric alignment

---

### 5. Autonomous Navigation

* Plan path from start to goal
* Execute trajectory using controller
* Validate in simulation and real robot

---

## ⚙️ Installation & Setup

### Prerequisites

* Ubuntu 20.04 / 22.04
* ROS 2 (Foxy / Humble)
* C++ compiler (gcc)

### Steps

```bash
# Clone repository
git clone https://github.com/your-username/3d-lidar-navigation.git

# Navigate to workspace
cd 3d-lidar-navigation

# Build the project
colcon build

# Source workspace
source install/setup.bash
```

---

## ▶️ Running the Project

### Launch Simulation

```bash
ros2 launch simulation.launch.py
```

### Run Mapping

```bash
ros2 launch mapping.launch.py
```

### Run Localization

```bash
ros2 launch localization.launch.py
```

### Start Navigation

```bash
ros2 launch navigation.launch.py
```

---

## 📊 Expected Results

* Accurate 3D maps of environment
* Real-time localization at sensor frame rate
* Smooth autonomous navigation
* Successful goal reaching in simulation and real-world setup

---

## 🚧 Challenges

* Real-time processing constraints
* Robust global localization
* Efficient data structures for 3D mapping
* System integration across modules

---

## 🔮 Future Work

* Multi-robot navigation
* Dynamic obstacle avoidance in 3D
* Sensor fusion (IMU + LiDAR)
* GPU acceleration for real-time performance

---

## 🤝 Contributions

Contributions are welcome! Feel free to fork the repo and submit pull requests.

---

## 📜 License

This project is licensed under the MIT License.

---

## 👤 Author

**Noel Innocent Joseph Sagayaraj**
Master’s Student in Mobile Robotics
University of Bonn
