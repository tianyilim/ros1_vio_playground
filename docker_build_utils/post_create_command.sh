#!/bin/bash

# This script is supposed to run INSIDE the docker container after it has been created.

set -e

# Build ORB_SLAM3
echo "Building ORB_SLAM3"
cd /ORB_SLAM3 && ./build.sh

# Build catkin_ws
echo "Building catkin_ws"
cd /catkin_ws && catkin build

# Add orbslam3 source to bashrc
echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc

echo "Finished."
