FROM nvidia/cuda:11.2.0-devel-ubuntu20.04 AS app

SHELL ["/bin/bash", "-c"]

# 3rd-party libs
RUN apt update && \
    DEBIAN_FRONTEND=noninteractive apt install -y \
    libpcl-dev \
    libeigen3-dev

# system dependencies
RUN apt update && \
    apt install -y \
    vim \
    netcat \
    curl \
    lsb-release \
    software-properties-common

# ROS
ENV ROS_DISTRO=noetic
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    apt update && \
    apt install -y ros-${ROS_DISTRO}-ros-base

# ROS additional packages
RUN apt update && \
    apt install -y \
    ros-${ROS_DISTRO}-rosbridge-server \
    ros-${ROS_DISTRO}-tf2-web-republisher \
    ros-${ROS_DISTRO}-pcl-conversions

RUN mkdir -p /catkin_ws/src

COPY entrypoint.sh /entrypoint.sh
COPY .bashrc /root/.bashrc

ENTRYPOINT ["/entrypoint.sh"]



#FROM ubuntu:18.04 AS visualization
#RUN apt update && \
#    apt install -y python3-dev && \
#    ln -sf /usr/bin/python3 /usr/bin/python \


