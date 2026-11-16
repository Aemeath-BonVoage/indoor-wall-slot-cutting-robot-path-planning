#!/bin/bash

# 停止并移除容器
docker stop ros2-humble-container
docker rm ros2-humble-container

# 撤销 X11 访问权限
xhost -local:docker
