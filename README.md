
# STR Robot: Setup and Build Guide

## 1. Prerequisites

- Ubuntu 22.04 LTS
- ROS 2 Humble Hawksbill
- Livox ROS Driver 2
- Micro-ROS Agent
- FAST-LIO / FAST-LIVO
- Nav2-based navigation stack

Install basic dependencies:

```bash
sudo apt update
sudo apt install -y \
    libpcl-dev \
    libeigen3-dev \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    build-essential \
    cmake \
    git

```

Source ROS 2 Humble:

```bash
source /opt/ros/humble/setup.bash

```

---

## 2. USB Device Setup

Create udev rule for STM32:

```bash
sudo nano /etc/udev/rules.d/99-ttbot.rules

```

Paste the following content:

```bash
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="ttbot_stm32", MODE="0666"

```

Apply udev rules:

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger

```

Check device:

```bash
ls /dev/ttbot_stm32

```

---

## 3. Build Livox ROS Driver 2

```bash
cd ~/STR_Robot/src/livox_ros_driver2
source /opt/ros/humble/setup.bash
./build.sh humble
source install/setup.bash

```

---

## 4. Build STR Robot Workspace

```bash
cd ~/STR_Robot
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash

```

---

## 5. Micro-ROS Agent

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttbot_stm32

```

---

## 6. Simulation

Run each command in a separate terminal.

### Terminal 1: Robot Bringup

```bash
cd ~/STR_Robot
source install/setup.bash
ros2 launch ttbot_bringup sim.launch.py

```

### Terminal 2: FAST-LIO Simulation

```bash
cd ~/STR_Robot
source install/setup.bash
ros2 launch fast_lio fast_lio_sim.launch.py

```

### Terminal 3: Navigation Simulation

```bash
cd ~/STR_Robot
source install/setup.bash
ros2 launch ttbot_navigation navigation_sim.launch.py

```

---

## 7. Real-World Deployment

Run the real robot system in the following order:

1. Launch Micro-ROS Agent.
2. Launch real-robot bringup.
3. Launch FAST-LIO / FAST-LIVO in real mode.
4. Launch the navigation stack with:

```bash
use_sim_time:=false

```

Before running navigation, check the map used by the Navigation map server.

---

## 8. Experimental Results

[cite_start]The system was evaluated in both simulation and real-world settings to evaluate path planning, path tracking, and autonomous navigation[cite: 406].

### 8.1. Simulation Experiments

[cite_start]The navigation architecture was validated in Gazebo using a non-holonomic robot model in a virtual environment constructed to closely resemble the real-world experimental setting[cite: 408, 409].

![Simulation Environment](path/to/fig4.png)

#### Path Planning Evaluation
[cite_start]The planner was evaluated on 20 randomized start-goal queries in simulation[cite: 411]. [cite_start]The improved A* algorithm generated shorter and smoother paths with fewer direction changes compared to the baseline A*[cite: 412].

* [cite_start]**Path length:** 25.52 m (Baseline A*) vs. 22.35 m (Improved A*)[cite: 416].
* [cite_start]**Total turning angle:** 3712.50° (Baseline A*) vs. 170.55° (Improved A*)[cite: 416].
* [cite_start]**Number of inflection points:** 79.45 (Baseline A*) vs. 4.55 (Improved A*)[cite: 416].
* [cite_start]**Planning time:** 221.15 ms (Baseline A*) vs. 330.54 ms (Improved A*)[cite: 416].

![Path Planning Comparison](path/to/fig6.png)

* [cite_start]**Video Demonstration:** https://youtu.be/LbHl0L6YVQk [cite: 413]

#### Path-Tracking Validation
[cite_start]Tracking performance was evaluated on lemniscate and square trajectories[cite: 417]. [cite_start]The proposed A-GMPC achieved lower Root Mean Square Error (RMSE) than the standard MPC on both trajectories, showing a more noticeable improvement on the lemniscate path[cite: 419].

* [cite_start]**Lemniscate Trajectory RMSE:** 0.1215 m (Standard MPC) vs. 0.0764 m (Proposed A-GMPC)[cite: 422].
* [cite_start]**Square Trajectory RMSE:** 0.2427 m (Standard MPC) vs. 0.1944 m (Proposed A-GMPC)[cite: 422].

![Simulation Path Tracking](path/to/fig5_ab.png)

#### Dynamic Obstacle Avoidance
Dynamic obstacle avoidance was evaluated under a hierarchical navigation framework. [cite_start]The global A* path was generated once, and a local avoidance module updated the reference path online using $C^2$-smooth curves to react to moving obstacles[cite: 423, 604].

* [cite_start]**Video Demonstration:** https://youtu.be/u9p_GGSCZNM [cite: 481]

---

### 8.2. Real-World Experiments

[cite_start]Physical experiments were conducted using a custom Ackermann-steered mobile robot[cite: 483].

![Physical Robot](path/to/fig7.png)

#### Real-World Path-Tracking Performance
[cite_start]The proposed architecture was deployed on the physical robot to verify sim-to-real consistency[cite: 484]. [cite_start]A-GMPC consistently achieved lower RMSE than the standard MPC for all tested trajectories and speeds[cite: 487].

* **At 0.6 m/s Average Speed:** A-GMPC RMSE was 0.2349 m (Lemniscate) and 0.2588 m (Square); [cite_start]Standard MPC RMSE was 0.3194 m (Lemniscate) and 0.4260 m (Square)[cite: 528].
* **At 1.1 m/s Average Speed:** A-GMPC RMSE was 0.3048 m (Lemniscate) and 0.2777 m (Square); [cite_start]Standard MPC RMSE was 0.3281 m (Lemniscate) and 0.4413 m (Square)[cite: 528].
* **At 1.5 m/s Average Speed:** A-GMPC RMSE was 0.3582 m (Lemniscate) and 0.6182 m (Square); [cite_start]Standard MPC RMSE was 0.5149 m (Lemniscate) and 1.1475 m (Square)[cite: 528].

[cite_start]Both controllers executed well within the 30 Hz (33.3 ms) control loop [cite: 521-522].

![Runtime Statistics](path/to/fig8.png)

![Real-World Path Tracking](path/to/fig5_cd.png)

* [cite_start]**Video Demonstration:** https://youtu.be/eXZOD7MUVX8 [cite: 490]

#### Autonomous Navigation Demonstration
[cite_start]An end-to-end real-world navigation experiment validated the complete autonomy pipeline, successfully integrating onboard perception, mapping, path planning, and trajectory tracking[cite: 523, 525]. [cite_start]The system achieved smooth tracking behavior with a low tracking error of RMSE = 0.2624 m over a reference path length of 114.0290 m[cite: 537, 538, 560].

![Environmental Mapping](path/to/fig9.png)

![Trajectory Tracking Performance](path/to/fig10.png)

* [cite_start]**Video Demonstration:** https://youtu.be/2TXuBDscRR4 [cite: 529]

---

## 9. Sub-repositories

The core development stack of the STR Robot is modularized into the following specialized sub-repositories:

* **Hardware (PCB Design):** https://github.com/Nvinh5148/STR_PCB.git
* **Firmware (micro-ROS Workspace):** https://github.com/Nvinh5148/microros_ws.git

