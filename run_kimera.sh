#!/bin/bash
set -e

docker stop kimera || true
docker rm kimera || true

# UI permisions
XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
xhost +local:docker

# Create a new container. It attaches to the current terminal and should compile.
# TODO: Adapt the bind mounts to your needs.
docker run --interactive --tty \
    --privileged --net=host --ipc=host \
    --name="kimera" \
    -e "DISPLAY=$DISPLAY" \
    -e "QT_X11_NO_MITSHM=1" \
    -e "XAUTHORITY=$XAUTH" \
    -e ROS_IP=127.0.0.1 \
    --cap-add=SYS_PTRACE \
    -v $XSOCK:$XSOCK \
    -v /etc/group:/etc/group:ro \
    --mount type=bind,source="$(pwd)/Datasets",target=/Datasets \
    --mount type=bind,source="$(pwd)/code/hl_orbslam3_wrapper",target=/catkin_ws/src/hl_orbslam3_wrapper \
    --mount type=bind,source="$(pwd)/code/Kimera-VIO-ROS",target=/catkin_ws/src/Kimera-VIO-ROS \
    --mount type=bind,source="$(pwd)/user",target=/user \
    --mount type=bind,source="/mnt/ssd_2T",target=/mnt/ssd_2T \
    --mount type=bind,source="/mnt/ssd_4T",target=/mnt/ssd_4T \
    kimera:ubuntu20_noetic_cuda

# TODO-neat way to mount custom dataset folder?
