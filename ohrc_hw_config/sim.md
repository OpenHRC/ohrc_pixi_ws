

## Universal Robots

run the following command to start the UR5e gazebo simulation, which has been installed automatically with OpenHRC:


The target robot type `UR_TYPE` needs to be chosen from `{ur3, ur5, ur10, ur3e, ur5e, ur7e, ur10e, ur12e, ur16e, ur8long, ur15, ur18, ur20, ur30}`.


```bash
$ ros2 launch ur_simulation_gz ur_sim_control.launch.py ur_type:={UR_TYPE} initial_joint_controller:=forward_velocity_controller launch_rviz:=false
```



## Franka Research 3






## xARM





