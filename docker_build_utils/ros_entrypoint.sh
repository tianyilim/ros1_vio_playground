#!/bin/bash
set -e

# This seems to be automatically executed by the ros:noetic image. It kinda sucks that it's not documented.

# Build stuff
cd / && ./post_create_command.sh

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
exec "$@"
