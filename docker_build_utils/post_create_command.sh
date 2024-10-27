#!/bin/bash

# This script is supposed to run INSIDE the docker container after it has been created.

set -e

# TODO: This command causes the script to fail. So don't check it
# Ensure catkin stuff is sourced (and fail early if not)
source "/opt/ros/noetic/setup.bash"

# Install catkin dependencies
rm -rf /etc/ros/rosdep/sources.list.d/20-default.list && \
    sudo rosdep init && \
    rosdep update && \
    rosdep install --from-paths /catkin_ws/src --ignore-src -r -y

# Unzip ORB vocabulary if not present
cd /ORB_SLAM3/Vocabulary
if [ ! -f ORBvoc.txt ]; then
    tar -xf ORBvoc.txt.tar.gz
fi

# Build ORB_SLAM
cd /ORB_SLAM3
./build.sh

# Modify the catkin workspace to the specific location of the orbslam3 wrapper, and copy vocabulary over
cd /catkin_ws/src
sed -i 's/$ENV{HOME}\/Packages//' orb_slam3_ros_wrapper/CMakeLists.txt
cp /ORB_SLAM3/Vocabulary/ORBvoc.txt orb_slam3_ros_wrapper/config/ORBvoc.txt

cd /catkin_ws
catkin build -j

echo "Post-create command finished."
