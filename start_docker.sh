#!/bin/sh

docker run \
    --name aip_bringup \
    --privileged \
    -it \
    -e DISPLAY=$DISPLAY \
    -v $PWD/.vscode:/home/docker/ros2_ws/src/.vscode \
    -v $PWD/cms50dplus_driver:/home/docker/ros2_ws/src/cms50dplus_ros2_driver/cms50dplus_driver \
    -v $PWD/ros2:/home/docker/ros2_ws/src/cms50dplus_ros2_driver/ros2 \
    -v /dev:/dev  \
    --net host \
    --rm \
    --ipc host \
    cms50dplus_ros2_driver:humble
