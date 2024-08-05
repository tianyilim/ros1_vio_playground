#!/bin/bash
set -e

# checking if you have nvidia
if ! nvidia-smi | grep "Driver" 2>/dev/null; then
  echo "******************************"
  echo """It looks like you don't have nvidia drivers running. Consider running build_container_cpu.sh instead."""
  echo "******************************"
  while true; do
    read -p "Do you still wish to continue?" yn
    case $yn in
      [Yy]* ) make install; break;;
      [Nn]* ) exit;;
      * ) echo "Please answer yes or no.";;
    esac
  done
fi

# Ensure that ORB_SLAM3 dirs are empty
rm -rf ORB_SLAM3 orb_slam3_ros_wrapper

# Ensure that ORB_SLAM3 is checked out
git submodule update --init --recursive # should clone from latest orbslam3
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
    --mount type=bind,source="$(pwd)/ORB_SLAM3",target=/ORB_SLAM3 \
    --mount type=bind,source="$HOME/mt/large_scale_pgo",target=/large_scale_pgo \
    --mount type=bind,source="$(pwd)/orb_slam3_ros_wrapper",target=/catkin_ws/src/orb_slam3_ros_wrapper \
    orbslam3:ubuntu20_noetic_cuda
