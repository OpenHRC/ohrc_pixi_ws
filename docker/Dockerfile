FROM ros:humble-perception

ENV ROS_WORKSPACE=/ros2_ws

RUN apt update -q && \
    apt upgrade -y

RUN mkdir -p ${ROS_WORKSPACE}/src

WORKDIR ${ROS_WORKSPACE}/src
RUN git clone https://github.com/OpenHRC/OpenHRC.git -b ros2 --recursive

RUN . /opt/ros/humble/setup.sh && \
    rosdep update && \
    rosdep install -i -y --from-paths ./

WORKDIR ${ROS_WORKSPACE}
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# RUN apt install -y ros-humble-spacenav

COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]