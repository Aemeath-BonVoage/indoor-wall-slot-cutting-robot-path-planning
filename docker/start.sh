#!/bin/bash

# 允许 Docker 容器访问 X11 显示
xhost +local:docker

# 检查容器是否已存在
if [ "$(docker ps -aq -f name=ros2-humble-container)" ]; then
    # 如果容器存在但未运行，启动它
    if [ "$(docker ps -q -f name=ros2-humble-container)" ]; then
        echo "容器已在运行"
    else
        echo "启动已存在的容器"
        docker start ros2-humble-container
    fi
else
    # 创建并启动新容器
    echo "创建新容器"
    docker run -itd \
        --name ros2-humble-container \
        --network host \
        --privileged \
        -v "$(pwd)/../src:/workspace/src" \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -e DISPLAY=${DISPLAY} \
        -e QT_X11_NO_MITSHM=1 \
        -e LIBGL_ALWAYS_SOFTWARE=1 \
        --device /dev/dri \
        ros2-humble-dev \
        /bin/bash
fi

# 进入容器
docker exec -it ros2-humble-container bash
