
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
