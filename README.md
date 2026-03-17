
### 4. USB Device Setup (Udev Rules)
```bash
sudo nano /etc/udev/rules.d/99-ttbot.rules
```
Copy and paste the following content into the file:
```bash
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="0001", SYMLINK+="ttbot_imu", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="ttbot_stm32", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a8", SYMLINK+="ttbot_gps", MODE="0666"
```
Apply and verify
``` bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```
### 5. IMU config (1 on 2 on)
```bash
stty -F /dev/ttbot_imu 460800 cs8 -cstopb -parenb -echo
echo -e '$DATOP 0000\r' > /dev/ttbot_imu
echo -e '$DATOP 1111\r' > /dev/ttbot_imu
echo -e '$BRATE 3\r' > /dev/ttbot_imu
```

source /home/vinh/microros_ws/install/local_setup.bash

ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0


# Data Logging & Analysis
## 1. Rosbag
Chạy lệnh này trên terminal của Robot trước khi cho xe chạy
```bash
mkdir -p ~/bag_files
cd ~/bag_files

ros2 bag record -a -o data_1
```

```bash
ros2 launch fast_lio mapping.launch.py config_file:=velodyne_sim.yaml use_sim_time:=true

ros2 launch slam_toolbox online_async_launch.py \
slam_params_file:=/home/uylegia/ttbot_ws/src/ttbot_mapping/config/mapper_params.yaml \
use_sim_time:=True
```

pcl_viewer /home/vinh/ttbot_ws/src/FAST-LIVO2/Log/PCD/all_raw_points.pcd
