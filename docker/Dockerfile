FROM ubuntu:18.04 AS clion-archive
COPY clion.tar.gz /clion.tar.gz
RUN mkdir -p /clion && \
    tar -xzf /clion.tar.gz -C /clion && \
    rm -r /clion.tar.gz



FROM ros:noetic-ros-core AS rc-4pcs

SHELL ["/bin/bash", "-c"]

RUN apt update && \
    DEBIAN_FRONTEND=noninteractive apt install -y \
    ros-noetic-rosbridge-server \
    libpcl-dev \
    libeigen3-dev

RUN apt update && \
    apt install -y \
    vim \
    netcat

RUN rm /ros_entrypoint.sh && \
    mkdir -p /catkin_ws/src

COPY entrypoint.sh /entrypoint.sh
COPY .bashrc /root/.bashrc
COPY --from=clion-archive /clion /clion

ENTRYPOINT ["/entrypoint.sh"]



#FROM ubuntu:18.04 AS visualization
#RUN apt update && \
#    apt install -y python3-dev && \
#    ln -sf /usr/bin/python3 /usr/bin/python \


