FROM osrf/ros:humble-desktop-full

# 设置环境变量
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# 安装基础工具和 ROS2 依赖
RUN apt-get update && apt-get install -y \
    git \
    vim \
    wget \
    curl \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros \
    && rm -rf /var/lib/apt/lists/*

# 创建工作空间
RUN mkdir -p /workspace/src
WORKDIR /workspace

# 设置 ROS2 环境
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi" >> ~/.bashrc
RUN echo "export GAZEBO_MODEL_DATABASE_URI=''" >> ~/.bashrc

CMD ["/bin/bash"]
