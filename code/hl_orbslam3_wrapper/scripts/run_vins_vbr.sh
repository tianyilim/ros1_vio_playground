#!/bin/bash

# This is the main runner script for all VINS VBR datasets.
# Specify IMU_ON and LC_ON flags

# if less than 3 arguments, print usage and exit
if [ "$#" -lt 2 ]; then
    echo "Usage: $0 <IMU_ON> <LC_ON>"
    echo "Example: $0 true false"
    exit 1
fi

IMU_ON=$1
LC_ON=$2

set -e

if [ "$LC_ON" = true ]; then
    echo "Loop Closure is ON"
    LAUNCHFILE="vbr-vins-stereo-lc.launch"
else
    LC_ON=false
    echo "Loop Closure is OFF"
    LAUNCHFILE="vbr-vins-stereo.launch"
fi

if [ "$IMU_ON" = true ]; then
    echo "IMU is ON"
    CONFIG_PATH="/catkin_ws/src/hl_orbslam3_wrapper/cfg/vins-fusion/vbr-vins-stereo.yaml"
else
    IMU_ON=false
    echo "IMU is OFF"
    CONFIG_PATH="/catkin_ws/src/hl_orbslam3_wrapper/cfg/vins-fusion/vbr-vins-stereo-no-imu.yaml"
fi

# List all train rosbags we have
VINS_OUTPUT_PATH="/home/tony-ws1/output"
BASE_PATH="/mnt/ssd_4T/tianyi_data/vbr/vbr_slam"
ENVIRONMENTS=(
    "colosseo colosseo_train0"
    "campus campus_train0"
    "campus campus_train1"
    "ciampino ciampino_train0"
    "ciampino ciampino_train1"
    "diag diag_train0"
    "pincio pincio_train0"
    "spagna spagna_train0"
)

mkdir -p "$VINS_OUTPUT_PATH"

for ENV in "${ENVIRONMENTS[@]}"; do
    # Iterate through subfolders
    IFS=' ' read -r -a array <<<"$ENV"
    SCENE=${array[0]}
    SUBFOLDER=${array[1]}

    ROSBAG_PATH=$BASE_PATH/$SCENE/$SUBFOLDER/$SUBFOLDER.bag
    OUTPUT=/user/vins-"$SUBFOLDER"-LC_"$LC_ON"-IMU_"$IMU_ON"
    mkdir -p "$OUTPUT"

    /usr/bin/time -o "$OUTPUT"/timing.txt roslaunch hl_orbslam3_wrapper "$LAUNCHFILE" rosbag_path:="$ROSBAG_PATH" config_path:="$CONFIG_PATH"

    mv "$VINS_OUTPUT_PATH"/* "$OUTPUT"
done
