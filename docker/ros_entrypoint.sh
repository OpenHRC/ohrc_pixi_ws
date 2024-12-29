#!/bin/bash
set -e

# setup ros environment
source "/ros2_ws/install/setup.bash" --

function get_started () {
    marker_teleoperation_ur5e
}

function marker_teleoperation_ur5e () {
    ur5e_sim & marker_teleoperation
}

function joy_topic_teleoperation_ur5e () {
    ur5e_sim & joy_topic_teleoperation
}

function ur5e_sim(){
    ros2 launch ur_simulation_gz ur_sim_control.launch.py initial_joint_controller:=forward_velocity_controller launch_rviz:=false
}

function marker_teleoperation(){
    ros2 launch ohrc_teleoperation marker_teleoperation.launch.py
}

function joy_topic_teleoperation(){
    ros2 launch ohrc_teleoperation joy_topic_teleoperation.launch.py
}

$@

