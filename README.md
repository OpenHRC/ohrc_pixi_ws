![GitHub release (latest by date)](https://img.shields.io/github/v/release/Automation-Research-Team/OpenHRC)
[![ROS build workflow](https://github.com/Automation-Research-Team/OpenHRC/actions/workflows/build.yaml/badge.svg)](https://github.com/Automation-Research-Team/OpenHRC/actions/workflows/build.yaml)
![GitHub](https://img.shields.io/github/license/Automation-Research-Team/OpenHRC)
# OpenHRC
#### Open Human-Robot Collaboration/Cooperation Library

Aiming to enhance research on human-robot interaction, we develop this open ROS package for offering easy implementation of HRC software such as interaction and teleoperation.
OpenHRC includes some tools for HRC like a robot controller for multiple robots with (self-)collision avoidance, human skeleton detection, imitation learning and so on. We hope this package helps you implement your HRC ideas instantly.

[Documentation by Doxygen](https://Automation-Research-Team.github.io/OpenHRC/) is under construction.

## Requirements
OpenHRC has been developed and tested in the following environments:
- Development Environment: Ubuntu 22.04 (ROS2 Humble)

**If you want to use ROS1 version, please checkout the `noetic` branch, which is no longer maintained.**




## Quick Start with Docker
You can quickly test UR5e + marker teleoperation on docker environment.
We use [rocker](https://github.com/osrf/rocker) here to simply run the docker container with GUI support.

### Build
```bash
$ git clone https://github.com/Automation-Research-Team/OpenHRC.git -b ros2
$ docker build -t openhrc:ros2 docker/.

$ sudo apt install python3-rocker # if you don't have rocker. You can also use "pip install rocker".
```

### Run
Now you can control the end-effector of UR5e with an interactive marker.
```bash
$ rocker --x11 openhrc:ros2 marker_teleoperation_ur5e 
```

<!-- If you have 3D mouse (spacenav), you can also use it for teleoperation.
```bash
$ rocker --x11 openhrc:humble joy_topic_teleoperation_ur5e
``` -->


## Native Installation

Step-by-step instructions for installing OpenHRC on your local machine.
In the following instruction, the ros2 workspace directory is assumed to be `~/ros2_ws` on host.


### Clone the Source Code and install dependencies
```bash
$ mkdir -p ~/ros2_ws/src
$ cd ~/ros2_ws/src
$ git clone https://github.com/Automation-Research-Team/OpenHRC.git -b ros2 --recursive
$ rosdep update && rosdep install -i -y --from-paths ./ 
```


### Build

```bash
$ cd ~/ros2_ws
$ colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```



## Getting Started
You can first try the teleoperation node with interactive markers for UR5e.

First, run the following command to start the UR5e simulation, which has been installed automatically with OpenHRC:
```bash
$ ros2 launch ur_simulation_gz ur_sim_control.launch.py initial_joint_controller:=forward_velocity_controller launch_rviz:=false
```

Then, run the following command to start the teleoperation node on another terminal:
```bash
$ source ~/ros2_ws/install/setup.bash
$ ros2 launch ohrc_teleoperation marker_teleoperation.launch.py
```


If you have a 3D mouse (spacenav), you can also use it for teleoperation instead.
```bash
$ source ~/ros2_ws/install/setup.bash
$ sudo apt install ros-humble-spacenav # if you don't have spacenav package
$ ros2 launch ohrc_teleoperation joy_topic_teleoperation.launch.py
```


## Controller Structure
*Under development

see [ohrc_control/README.md](./ohrc_control/README.md)


## Tutorials
*Under development
1. Teleoperation library: [ohrc_teleoperation/README.md](./ohrc_teleoperation)
2. Imitation Learning library: [ohrc_imitation_learning/README.md](./ohrc_imitation_learning)



## Citation

If you use this package in your academic research, we would appreciate it if you could cite the following paper.
```
@software{openhrc,
  author = {Shunki Itadera},
  title = {OpenHRC},
  url = {https://github.com/Automation-Research-Team/OpenHRC},
  year = {2024},
}
```


## License
This software is released under dual license of LGPL-v2.1 and individual license.
About the individual license, please make a contact to the author.

## Author
Shunki Itadera (https://itadera.github.io/) - Researcher at ART, ICPS, AIST

We welcome any feedback and contributions. Please feel free to contact us if you have any questions or comments.

Besides, we are looking for research collaborators and students who are interested in Human-Robot Interaction using OpenHRC. If you are interested, please send me an email.
