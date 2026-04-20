FROM osrf/ros:humble-desktop
RUN for retry in $(seq 10); do apt-get update -o Acquire::https::Verify-Peer="false" -o Acquire::https::Verify-Host="false" -o Acquire::AllowInsecureRepositories="true" && break; done

RUN apt-get update \
  && apt-get upgrade -y \
  && apt-get clean

RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    lsb-release \
    python3-colcon-ros \
    && apt-get clean

RUN apt-get upgrade -y
RUN apt-get update && apt-get install sudo -y

RUN apt-get update && apt-get install -y \
        ros-humble-ros2-control \
        ros-humble-ros2-controllers
RUN apt-get update && apt-get install -y \
        ros-humble-moveit \
        ros-humble-position-controllers \
        ros-humble-gripper-controllers
RUN apt-get update && apt-get install -y \
        ros-humble-effort-controllers \
        ros-humble-ackermann-msgs
RUN apt-get update && apt-get install -y \
        ros-humble-joint-trajectory-controller \
        ros-humble-joint-state-publisher
RUN apt-get update && apt-get install -y \
        ros-humble-joint-state-publisher-gui \
        ros-humble-slam-toolbox
RUN apt-get update && apt-get install -y \
        ros-humble-navigation2 \
        ros-humble-laser-proc
        
        
RUN apt-get update && apt-get install -y mc
RUN apt-get update && apt-get install -y htop
RUN apt-get update && apt-get install -y python3.9

RUN rm /bin/sh && ln -s /bin/bash /bin/sh

RUN apt-get update && apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    python3-colcon-common-extensions

RUN pip install --upgrade pip
RUN pip install jupyter numpy pandas matplotlib


RUN apt-get update && apt-get install -y \
    curl \
    git \
    && rm -rf /var/lib/apt/lists/*

RUN apt update && apt install -y \
     nautilus \
     jupyter-

RUN apt-get update && apt-get install -y \
     ros-humble-image-transport-plugins \
     ros-humble-camera-info-manager


RUN apt-get update && apt-get install -y wget gnupg2 lsb-release \
    && wget -qO- https://cyberbotics.com/Cyberbotics.asc | gpg --dearmor -o /usr/share/keyrings/cyberbotics-archive-keyring.gpg \
    && echo "deb [arch=amd64 signed-by=/usr/share/keyrings/cyberbotics-archive-keyring.gpg] https://cyberbotics.com/debian/ binary-amd64/" \
       > /etc/apt/sources.list.d/cyberbotics.list \
    && apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y webots \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    x11-apps \
    && apt-get clean

RUN apt-get update && apt-get install -y \
    ros-humble-webots-ros2 \
    ros-humble-smach \
    ros-humble-smach-ros \
    && apt-get clean
    
RUN apt-get update && apt-get install -y \
    ros-humble-cv-bridge \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-std-msgs \
    && rm -rf /var/lib/apt/lists/*

RUN pip install --upgrade pip


RUN apt-get remove -y python3-sympy || true

RUN pip install --upgrade pip
RUN pip install --upgrade --ignore-installed sympy
RUN pip install "numpy<2.0" opencv-python pillow ultralytics

RUN apt-get update && apt-get install -y ros-humble-smach ros-humble-executive-smach


# Добавляем инициализацию ROS и переменные окружения в .bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "export WEBOTS_HOME=/usr/local/webots" >> ~/.bashrc && \
    echo "export ROS2_WEBOTS_HOME=/usr/local/webots" >> ~/.bashrc && \
    echo "export USER=\$(whoami)" >> ~/.bashrc

