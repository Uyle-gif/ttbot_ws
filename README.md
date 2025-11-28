# ttbot_ws

A ROS2 Humble workspace for an autonomous Ackermann-steering robot running on **Ubuntu 22.04**.  
This project includes control, sensor processing, localization, and system bringup for a car-like robotic platform.

---

REF: https://github.com/Nvinh5148/microros_ws

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
- Stanley controller, MPC controller
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
## 🧩 Prerequisites
### 1. Ubuntu and ROS
Ubuntu >= 20.04

ROS2 >= Foxy (Recommend to use ROS2-Humble).

Installation guide: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

_Note: Please install the required ROS 2 packages._
### 2. OSQP
```bash
cd ~
git clone https://github.com/osqp/osqp.git
cd osqp
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```
### 3. Eigen
```bash
cd ~
git clone https://gitlab.com/libeigen/eigen.git
cd eigen
mkdir build && cd build
cmake ..
sudo make install
```


### 4. USB Device Setup (Udev Rules)
```bash
sudo nano /etc/udev/rules.d/99-ttbot.rules
```
Copy and paste the following content into the file:
```bash
# --- IMU SENSOR ---
# Create fixed symlink: /dev/ttbot_imu
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="0001", SYMLINK+="ttbot_imu", MODE="0666"

# --- STM32 MICRO-CONTROLLER ---
# Create fixed symlink: /dev/ttbot_stm32
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="ttbot_stm32", MODE="0666"
```
Apply and verify
``` bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### 5. IMU config
```bash
echo -ne '$DATOP 0000\r' > /dev/ttbot_imu
echo -ne '$DATOP 1111\r' > /dev/ttbot_imu
echo -ne '$BRATE 3\r' > /dev/ttbot_imu
```


## ⚙️ Build Instructions (ROS2 Humble)

### 1. Source ROS2 
```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
source /opt/ros/humble/setup.bash
```

### 2. Build & source the workspace 
```bash
cd ~/Desktop/ttbot_ws
colcon build 
source install/setup.bash
```
### 3. Run the full system
```bash
ros2 launch ttbot_bringup ttbot_full_system.launch.py
```
### 4. Run Stanley_controller
```bash
ros2 launch ttbot_controller stanley.launch.py
```
### 5. Run mpc_controller
```bash
ros2 launch ttbot_controller mpc.launch.py
```
### 6. Pub path for MPC
```bash
    ros2 run ttbot_controller path_publisher --ros-args -p path_file:=path_u_to_S.csv
```


### 7. Run imu node 
```bash
sudo usermod -aG dialout $USER

logout then login again 
ros2 run adis16488_driver adis16488_node --ros-args -p port:=/dev/ttbot_imu -p baudrate:=460800
```





