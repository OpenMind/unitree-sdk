FROM ros:humble-ros-base-jammy AS base

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y \
    software-properties-common \
    python3-pip \
    python3.10 \
    python3.10-distutils \
    python3.10-dev \
    build-essential \
    cmake \
    ros-humble-ament-cmake \
    ros-humble-ament-cmake-core \
    ros-humble-ament-package \
    ros-humble-rosidl-generator-dds-idl \
    ros-humble-foxglove-bridge \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-joy \
    ros-humble-builtin-interfaces \
    ros-humble-std-msgs \
    ros-humble-geometry-msgs \
    ros-humble-rosidl-default-generators \
    ros-humble-rosidl-default-runtime \
    ros-humble-rosidl-generator-c \
    ros-humble-rosidl-typesupport-c \
    ros-humble-rosidl-typesupport-cpp \
    libportaudio2 \
    x11-apps \
    libsm6

RUN update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.10 1 \
    && update-alternatives --set python3 /usr/bin/python3.10

RUN python3 -m pip install --upgrade pip

ENV ROS_DISTRO=humble \
    ROS_ROOT=/opt/ros/${ROS_DISTRO} \
    ROS_PYTHON_VERSION=3 \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    DEBIAN_FRONTEND=noninteractive \
    SHELL=/bin/bash

RUN mkdir /app
WORKDIR /app

RUN git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x
RUN cd cyclonedds && mkdir build install && cd build

WORKDIR /app/cyclonedds/build
RUN cmake .. -DCMAKE_INSTALL_PREFIX=../install -DBUILD_EXAMPLES=ON
RUN cmake --build . --target install
ENV CYCLONEDDS_HOME=/app/cyclonedds/install \
    CMAKE_PREFIX_PATH=/app/cyclonedds/install

WORKDIR /app

RUN mkdir -p /app/unitree_go2_ros2_sdk
COPY . /app/unitree_go2_ros2_sdk

WORKDIR /app/unitree_go2_ros2_sdk

RUN rosdep install -y --ignore-src --from-paths . -r
RUN pip install -r requirements.txt --force-reinstall --no-deps
RUN source /opt/ros/humble/setup.bash && colcon build

RUN echo '#!/bin/bash' > /entrypoint.sh && \
    echo 'set -e' >> /entrypoint.sh && \
    echo '' >> /entrypoint.sh && \
    echo '# Source ROS environment' >> /entrypoint.sh && \
    echo 'source /opt/ros/humble/setup.bash' >> /entrypoint.sh && \
    echo 'source /app/unitree_go2_ros2_sdk/install/setup.bash' >> /entrypoint.sh && \
    echo '' >> /entrypoint.sh && \
    echo '# If no arguments provided, run default command' >> /entrypoint.sh && \
    echo 'if [ $# -eq 0 ]; then' >> /entrypoint.sh && \
    echo '    exec ros2 launch go2_sdk slam_launch.py' >> /entrypoint.sh && \
    echo 'else' >> /entrypoint.sh && \
    echo '    exec "$@"' >> /entrypoint.sh && \
    echo 'fi' >> /entrypoint.sh && \
    chmod +x /entrypoint.sh

# RUN echo "deb [trusted=yes] https://download.eclipse.org/zenoh/debian-repo/ /" | sudo tee -a /etc/apt/sources.list > /dev/null
# RUN apt update && apt install zenoh-bridge-ros2dds -y

ENTRYPOINT ["/entrypoint.sh"]
