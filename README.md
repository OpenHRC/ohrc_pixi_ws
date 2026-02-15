# ohrc_pixi_ws

This is as setup file for [OpenHRC](https://github.com/OpenHRC/OpenHRC) on [Pixi](https://pixi.prefix.dev/latest/).

We have tested on
- macOS Tahoe
- Ubuntu 22.04

## 1. Install Pixi
Follow the official instruction [WEB](https://pixi.prefix.dev/latest/installation/)

If you install it on macOS or Linux
```bash
curl -fsSL https://pixi.sh/install.sh | sh
```

## 2. Clone this repository
```bash
git clone https://github.com/OpenHRC/ohrc_pixi_ws.git --recursive
```

## 3. Install Dependencies
```bash
cd ohrc_pixi_ws
pixi install
```

## 4. Clone and build OpenHRC
```bash
pixi run clone && pixi run build
```
which is alias of `pixi run colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release`.

## 5. Run an example
Open two terminals and `cd` to ohrc_pixi_ws directory.

### 1st terminal (run ur5e simulation)
```bash
pixi run ur5e_gz
```
which is alias of `pixi run ros2 launch ur_simulation_gz ur_sim_control.launch.py initial_joint_controller:=forward_velocity_controller launch_rviz:=false gazebo_gui:=false`.

### 3nd terminal (execute marker teleoperation)
```bash
pixi run ohrc_marker_teleoperation
``` 
which is alias of `pixi run ros2 launch ohrc_teleoperation marker_teleoperation.launch.py`.