ARG ROS_DISTRO=humble
 
FROM arm64v8/ros:${ROS_DISTRO}-ros-base as base
ENV ROS_DISTRO=${ROS_DISTRO}
SHELL ["/bin/bash", "-c"]

RUN set -euo pipefail
ENV DEBIAN_FRONTEND=noninteractive

# Get Ubuntu packages
RUN apt-get update && apt-get install -y \
    build-essential \
    curl \
    tmux \
    git \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Install Rust
RUN apt-get update && curl https://sh.rustup.rs -sSf | bash -s -- -y
ENV PATH="/root/.cargo/bin:${PATH}"

# Compile and install tpi
RUN git clone --depth 1 --branch v1.0.6 https://github.com/turing-machines/tpi.git && \
    cd tpi && cargo install tpi

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
