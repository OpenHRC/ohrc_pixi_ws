FROM ros:humble-perception

ENV ROS_WORKSPACE=/ros2_ws

RUN apt update -q && \
    apt upgrade -y

RUN mkdir -p ${ROS_WORKSPACE}/src


WORKDIR ${ROS_WORKSPACE}/src
RUN git clone https://github.com/Automation-Research-Team/OpenHRC.git -b ros2 --recursive && \
    git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation.git -b humble

RUN . /opt/ros/humble/setup.sh && \
    rosdep update && \
    rosdep install -i -y --from-paths ./

WORKDIR ${ROS_WORKSPACE}
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]