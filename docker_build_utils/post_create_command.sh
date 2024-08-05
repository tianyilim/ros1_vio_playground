#!/bin/bash

# This script is supposed to run INSIDE the docker container after it has been created.

set -e

# Ensure catkin stuff is sourced (and fail early if not)
source "/opt/ros/noetic/setup.bash"
catkin list

# Build ORB_SLAM3
echo "Building ORB_SLAM3"
cd /ORB_SLAM3 && ./build.sh

# Ensure that ros dependencies are all installed
rm -rf /etc/ros/rosdep/sources.list.d/20-default.list && \
sudo rosdep init && \
rosdep update && \
rosdep install --from-paths /catkin_ws/src --ignore-src -r -y

# Build catkin_ws
echo "Building catkin_ws"
cd /catkin_ws && catkin build
# Add orbslam3 source to bashrc
echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc

echo "Finished."
