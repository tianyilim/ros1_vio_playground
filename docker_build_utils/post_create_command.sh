#!/bin/bash

# This script is supposed to run INSIDE the docker container after it has been created.

set -e

# Ensure catkin stuff is sourced (and fail early if not)
source "/opt/ros/noetic/setup.bash"
catkin list

echo "Post-create command finished."
