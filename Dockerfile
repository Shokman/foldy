ARG ROS_DISTRO=humble
 
FROM arm64v8/ros:${ROS_DISTRO}-ros-base as base
ENV ROS_DISTRO=${ROS_DISTRO}
SHELL ["/bin/bash", "-c"]

FROM base AS overlay
 
RUN mkdir -p /root/ws
COPY ./src /root/ws/src
WORKDIR /root/ws/src
RUN vcs import < neato.repos
 
WORKDIR /root/ws
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
 && apt-get update -y \
 && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y \
 && colcon build --symlink-install

COPY ./entrypoint.sh /
RUN chmod +x  /entrypoint.sh
ENTRYPOINT [ "/entrypoint.sh" ]
CMD ["bash"]
