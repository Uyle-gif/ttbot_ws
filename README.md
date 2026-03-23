### OSQP, EIGEN
### USB Device Setup (Udev Rules)
```bash
sudo nano /etc/udev/rules.d/99-ttbot.rules
```
Copy and paste the following content into the file:
```bash
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="ttbot_stm32", MODE="0666"
```
Apply and verify
``` bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

<!-- ### IMU config (1 on 2 on)
```bash
stty -F /dev/ttbot_imu 460800 cs8 -cstopb -parenb -echo
echo -e '$DATOP 0000\r' > /dev/ttbot_imu
echo -e '$DATOP 1111\r' > /dev/ttbot_imu
echo -e '$BRATE 3\r' > /dev/ttbot_imu
``` -->
### Micro ROS-Agent

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttbot_stm32
```

