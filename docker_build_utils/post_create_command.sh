#!/bin/bash

# This script is supposed to run INSIDE the docker container after it has been created.

set -e

# TODO: This command causes the script to fail. So don't check it
# Ensure catkin stuff is sourced (and fail early if not)
source "/opt/ros/noetic/setup.bash"
# catkin list

# Modify the catkin workspace to the specific location of the orbslam3 wrapper, and copy vocabulary over
cd /catkin_ws/src
sed -i 's/$ENV{HOME}\/Packages//' orb_slam3_ros_wrapper/CMakeLists.txt
cp /ORB_SLAM3/Vocabulary/ORBvoc.txt orb_slam3_ros_wrapper/config/ORBvoc.txt

echo "Post-create command finished."
