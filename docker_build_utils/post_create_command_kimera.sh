#!/bin/bash

# This script is supposed to run INSIDE the docker container after it has been created.

# set -e

# TODO: This command causes the script to fail. So don't check it
# Ensure catkin stuff is sourced (and fail early if not)
source "/opt/ros/noetic/setup.bash"

# Install catkin dependencies
rm -rf /etc/ros/rosdep/sources.list.d/20-default.list && \
    sudo rosdep init && \
    rosdep update && \
    rosdep install --from-paths /catkin_ws/src --ignore-src -r -y

# Do KimeraVIO ROS Wrapper installation
cd /catkin_ws/
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release -DGTSAM_TANGENT_PREINTEGRATION=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON

cd /catkin_ws/src
wstool init
wstool merge Kimera-VIO-ROS/install/kimera_vio_ros_https.rosinstall # install Kimera dependencies
wstool update   # download and update repos
rosdep install --from-paths . --ignore-src -r -y

# Checkout correct verison of GTSAM
cd /catkin_ws/src/gtsam
git checkout 4.2.0

# Install CV2
cd /catkin_ws/src
git clone https://github.com/ethz-asl/opencv3_catkin.git

# Build Everything
cd /catkin_ws
catkin build -j

echo "Post-create command finished."
