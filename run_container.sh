#!/bin/bash
set -e

# UI permisions
XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
xhost +local:docker

# Create a new container. It attaches to the current terminal and should compile.
docker run --interactive --tty \
    --privileged --net=host --ipc=host \
    --name="orbslam3" \
    --gpus=all \
    -e "DISPLAY=$DISPLAY" \
    -e "QT_X11_NO_MITSHM=1" \
    -e "XAUTHORITY=$XAUTH" \
    -e ROS_IP=127.0.0.1 \
    --cap-add=SYS_PTRACE \
    -v $XSOCK:$XSOCK \
    -v /etc/group:/etc/group:ro \
    --mount type=bind,source="$(pwd)/Datasets",target=/Datasets \
    --mount type=bind,source="$(pwd)/code",target=/catkin_ws/src/code \
    --mount type=bind,source="$(pwd)/user",target=/user \
    --mount type=bind,source="$HOME/mt/large_scale_pgo",target=/Datasets/large_scale_pgo \
    orbslam3:ubuntu20_noetic_cuda

# TODO-neat way to mount custom dataset folder?
