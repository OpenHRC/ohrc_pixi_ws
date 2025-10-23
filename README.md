![GitHub release (latest by date)](https://img.shields.io/github/v/release/OpenHRC/OpenHRC)
![GitHub](https://img.shields.io/github/license/OpenHRC/OpenHRC)


<p align="center">
  <img src="docs/logo_openhrc.jpg" alt="OpenHRC Logo" width="600"/>
</p>


# OpenHRC: Open Human-Robot Collaboration Library

Aiming to enhance research on human-robot interaction, we develop this open ROS2 package for offering easy implementation of HRC software such as interaction and teleoperation.
OpenHRC includes some tools for HRC like a robot controller for multiple robots with (self-)collision avoidance, human skeleton detection, imitation learning and so on. We hope this package helps you implement your HRC ideas instantly.

[Documentation by Doxygen](https://openhrc.github.io/OpenHRC/) is under construction.

## Requirements
OpenHRC has been developed and tested in the following environments:
- Development Environment: Ubuntu 22.04 (ROS 2 Humble)


#### CI Build Status
| ROS 2 Distribution | Humble | Jazzy |
|--------------------|--------|-------|
| Build Status | [![humble-build](https://github.com/OpenHRC/OpenHRC/actions/workflows/humble-build.yaml/badge.svg)](https://github.com/OpenHRC/OpenHRC/actions/workflows/humble-build.yaml) | [![jazzy-build](https://github.com/OpenHRC/OpenHRC/actions/workflows/jazzy-build.yaml/badge.svg)](https://github.com/OpenHRC/OpenHRC/actions/workflows/jazzy-build.yaml) |

**If you want to use ROS1 version, please checkout the [`noetic`](https://github.com/OpenHRC/OpenHRC/tree/noetic) branch, which is no longer maintained.**




## Quick Start with Docker
See [quick_start_w_docker_guide](./docker/README.md).


## Native Installation

Step-by-step instructions for installing OpenHRC on your local machine.
In the following instruction, the ros2 workspace directory is assumed to be `~/ros2_ws` on host.


### Clone the Source Code and install dependencies
```bash
$ mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
$ git clone https://github.com/OpenHRC/OpenHRC.git -b ros2 --recursive
$ rosdep update && rosdep install -i -y --from-paths ./ 
```


### Build

```bash
$ cd ~/ros2_ws
$ colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```



## Getting Started
You can first try the teleoperation node for UR5e.

First, run the following command to start the UR5e gazebo simulation, which has been installed automatically with OpenHRC:
```bash
$ ros2 launch ur_simulation_gz ur_sim_control.launch.py initial_joint_controller:=forward_velocity_controller launch_rviz:=false
```

We offer several teleoperation interfaces to control the robot remotely.

#### 1. Interactive Marker
You can try Interactive Marker on Rviz.
```bash
$ ros2 launch ohrc_teleoperation marker_teleoperation.launch.py
```

#### 2. keyboard
You can operate the robot via keyboard as well.
- Translation -> (UP/DOWN, LEFT/RIGHT, PageUP/PageDOWN)
- Rotation -> (NK8/NK2, NK4/NK6, NK+/NK-) NK:Numeric Keypad
```bash
$ ros2 launch ohrc_teleoperation joy_topic_teleoperation.launch.py device:=keyboard
```

#### 3. 3D mouse (spacenav)
If you have 3D mouse (spacenav), you can also use it.
```bash
$ sudo apt install ros-humble-spacenav # if you don't have spacenav package
$ ros2 launch ohrc_teleoperation joy_topic_teleoperation.launch.py device:=spacenav
```

## Controller Structure
*Under development

see [ohrc_control/README.md](./ohrc_control/README.md)


## Tutorials
*Under development
1. Teleoperation library: [./ohrc_teleoperation/README.md](./ohrc_teleoperation)
<!-- 2. Imitation Learning library: [ohrc_imitation_learning/README.md](./ohrc_imitation_learning) -->



## Citation

If you use this package in your academic research, we would appreciate it if you could cite the following paper.
```
@software{openhrc,
  author = {Shunki Itadera},
  title = {OpenHRC},
  url = {https://github.com/OpenHRC/OpenHRC},
  year = {2024},
}
```


## License
This software is released under MIT.

## Author
Shunki Itadera (https://itadera.github.io/) - Researcher at AIST, Japan

We welcome any feedback and contributions. Please feel free to contact us if you have any questions or comments.

Besides, we are looking for research collaborators and students who are interested in Human-Robot Interaction using OpenHRC. If you are interested, please send me a message.
