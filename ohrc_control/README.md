
# Controller Structure

Within the ROS (ROS 2) ecosystem, we offer two types of controller structures:
1. Topic-based controller  
2. Realtime controller (coming soon)



## 1. Topic-based Controller

This is the simplest way (in our opinion) to control a robot by taking advantage of ROS's pub/sub system.

If the robot can receive command topics such as joint velocity commands, this method can be used with minimal setup—typically just adjusting parameters like the robot's namespace and topic names.

However, this control scheme relies on ROS topic communication between the robot and controller nodes, and it does **not** guarantee real-time loop performance. In practice, this usually isn’t a problem. But for high-frequency force control (e.g., 1 kHz control for a Franka robot), this method may lead to unstable behavior.


## 2. Realtime Controller

This approach involves controlling the robot using a real-time kernel on the host PC.  
(It is not yet available, but we plan to support it.)

