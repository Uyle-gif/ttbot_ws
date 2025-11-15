# ttbot_ws

A ROS2 Humble workspace for an autonomous Ackermann-steering robot running on **Ubuntu 22.04**.  
This project includes control, sensor processing, localization, and system bringup for a car-like robotic platform.

---

## 🚗 Overview

The `ttbot_ws` workspace provides:

- Ackermann steering control  
- Rear-wheel speed control  
- Sensor processing (IMU, encoders, LiDAR depending on configuration)  
- Localization and odometry fusion  
- Robot URDF/Xacro description  
- Bringup launch system for ROS2  
- Clean structure for real robot deployment and simulation

Designed to run natively on **ROS2 Humble**.

---

## 📦 Packages

### **`ttbot_bringup`**
System launch and configuration package.

Includes:
- Robot bringup launch files  
- Controller + sensor launch  
- Parameter YAML files  
- Unified startup for real robot or simulation  

---

### **`ttbot_controller`**
Ackermann control module.

Features:
- Steering angle control  
- Speed control for drive wheels  
- Subscribes to `/cmd_vel` or `/ackermann_cmd`  
- Publishes odometry & TF transforms  
- Converts velocity commands to Ackermann steering commands

---

### **`ttbot_description`**
Robot model (URDF/Xacro).

Contains:
- Robot chassis & wheel model  
- Ackermann geometry  
- Sensor frames (LiDAR, IMU, etc.)  
- Visual + collision models  
- Ready for RViz2 & Gazebo simulation

---

### **`ttbot_localization`**
Localization module.

Provides:
- Odometry fusion (IMU + encoders)  
- EKF/UKF (with `robot_localization`)  
- TF map → odom → base_link  
- Stable localization for navigation

---


---

## ⚙️ Build Instructions (ROS2 Humble)

### 1. Source ROS2
```bash
source /opt/ros/humble/setup.bash
```

### 1. Build & source the workspace 
```bash
cd ~/ttbot_ws
colcon build 
source install/setup.bash
```
### 3. Run the full system
```bash
ros2 launch ttbot_bringup ttbot_full_system.launch.py
```







