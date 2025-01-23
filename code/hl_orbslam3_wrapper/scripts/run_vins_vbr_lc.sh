#!/bin/bash

set -e

# List all train rosbags we have
VINS_OUTPUT_PATH="/home/tony-ws1/output"
BASE_PATH="/mnt/ssd_4T/tianyi_data/vbr/vbr_slam"
ENVIRONMENTS=(
    "colosseo colosseo_train0"
    # "campus campus_train0"
    # "campus campus_train1"
    # "ciampino ciampino_train0"
    "ciampino ciampino_train1"
    "diag diag_train0"
    "pincio pincio_train0"
    "spagna spagna_train0"
)

CONFIG_PATH="/catkin_ws/src/hl_orbslam3_wrapper/cfg/vins-fusion/vbr-vins-stereo.yaml"

mkdir -p "$VINS_OUTPUT_PATH"

for ENV in "${ENVIRONMENTS[@]}"; do
    # Iterate through subfolders
    IFS=' ' read -r -a array <<<"$ENV"
    SCENE=${array[0]}
    SUBFOLDER=${array[1]}

    ROSBAG_PATH=$BASE_PATH/$SCENE/$SUBFOLDER/$SUBFOLDER.bag
    OUTPUT=/user/vins-"$SUBFOLDER"

    /usr/bin/time -o "$OUTPUT"/timing.txt roslaunch hl_orbslam3_wrapper vbr-vins-stereo-lc.launch rosbag_path:="$ROSBAG_PATH" config_path:="$CONFIG_PATH"

    mkdir -p "$OUTPUT"
    mv "$VINS_OUTPUT_PATH"/* "$OUTPUT"
    # mv ~/.ros/VINS_KeyframeMemUsageKB.txt "$OUTPUT"
    # mv ~/.ros/VINS_KeyframeTrackTiming.txt "$OUTPUT"
done
