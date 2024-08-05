#!/bin/bash
set -e

# Ensure that ORB_SLAM3 dirs are empty
rm -rf ORB_SLAM3 orb_slam3_ros_wrapper

# Ensure that ORB_SLAM3 is checked out
git submodule update --init --recursive 
cd ORB_SLAM3/Vocabulary && tar -xvf ORBvoc.txt.tar.gz && cd - || exit 1 # Extract vocabulary
# Modify the catkin workspace to the specific location of the orbslam3 wrapper
sed -i 's/$ENV{HOME}\/Packages//' orb_slam3_ros_wrapper/CMakeLists.txt
# Copy vocabulary over
cp ORB_SLAM3/Vocabulary/ORBvoc.txt orb_slam3_ros_wrapper/config/ORBvoc.txt

# UI permisions
XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
xhost +local:docker

# Remove existing container
docker rm -f orbslam3 &>/dev/null

# Docker build
docker buildx build -t orbslam3:ubuntu20_noetic_cuda -f Dockerfile .

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
    --mount type=bind,source="$(pwd)/code",target=/code \
    --mount type=bind,source="$(pwd)/user",target=/user \
    --mount type=bind,source="$HOME/mt/large_scale_pgo",target=/code/large_scale_pgo \
    orbslam3:ubuntu20_noetic_cuda

# TODO-neat way to mount custom dataset folder?
