# Dev image for pick-and-place课设: Humble + Gazebo (gz) + Franka build deps.
# Build: docker compose build
# Run:   docker compose up -d && docker compose exec ros2-gazebo bash

FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-vcstool \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-ros-gz \
    ros-humble-gz-ros2-control \
    ros-humble-pinocchio \
    ros-humble-moveit \
    ros-humble-moveit-servo \
    ros-humble-moveit-resources-panda-moveit-config \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    ros-humble-cv-bridge \
    python3-opencv \
    python3-pip \
    python3-yaml \
    git \
    wget \
    && rm -rf /var/lib/apt/lists/*

COPY requirements.txt /tmp/robot-homework-ros-requirements.txt
RUN python3 -m pip install --no-cache-dir -r /tmp/robot-homework-ros-requirements.txt

WORKDIR /root/ros2_ws

# Default command keeps container alive; workspace is bind-mounted from host.
CMD ["tail", "-f", "/dev/null"]
