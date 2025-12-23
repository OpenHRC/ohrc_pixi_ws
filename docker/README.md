## Quick Start with Docker

You can quickly test UR5e + marker teleoperation on docker environment.

### Preparation
We use [rocker](https://github.com/osrf/rocker) here to simply run the docker container with GUI support.

```bash
$ sudo apt install python3-rocker # if you don't have rocker. You can also use "pip install rocker".
```


### Build Docker Image
```zsh
$ docker build -t openhrc:ros2 https://github.com/OpenHRC/OpenHRC.git\#ros2:docker
```
### Start Docker Container
Now you can control the end-effector of UR5e with an interactive marker.
```bash
$ rocker --x11 openhrc:ros2 marker_teleoperation_ur5e 
```
<!-- If you have Nvidia GPU, you can replace `--x11` with `--nvidia auto`. -->
<!-- If you have 3D mouse (spacenav), you can also use it for teleoperation.
```bash
$ rocker --x11 openhrc:humble joy_topic_teleoperation_ur5e
``` -->

If you control the robot using your keyboard,
```bash
$ rocker --x11 openhrc:ros2 keyboard_teleoperation_ur5e 
```
Note that you make sure that the small window named `keyboard` is active (selected).
- Translation -> (UP/DOWN, LEFT/RIGHT, PageUP/PageDOWN)
- Rotation -> (NK8/NK2, NK4/NK6, NK+/NK-) NK:Numeric Keypad