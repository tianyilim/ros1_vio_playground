#!/bin/bash

set -e

# List all train rosbags we have
VINS_OUTPUT_PATH="/home/tony-ws1/output"
BASE_PATH="/mnt/ssd_4T/tianyi_data/vbr/vbr_slam"
ENVIRONMENTS=(
    "campus"
    "ciampino"
    "colosseo"
    "diag"
    "pincio"
    "spagna"
)

CONFIG_PATH="/catkin_ws/src/hl_orbslam3_wrapper/cfg/vins-fusion/vbr-vins-stereo.yaml"

mkdir -p "$VINS_OUTPUT_PATH"

for ENV in "${ENVIRONMENTS[@]}"; do
    # Iterate through subfolders
    for SUBFOLDER in "$BASE_PATH"/"$ENV"/*; do
        echo "Iterating through $SUBFOLDER"
        BAG_NAME=$(basename "$SUBFOLDER")

        ROSBAG_PATH="$SUBFOLDER"/"$BAG_NAME".bag
        OUTPUT=/user/vins-"$BAG_NAME"
        mkdir -p "$OUTPUT"

        roslaunch hl_orbslam3_wrapper vbr-vins-stereo-lc.launch rosbag_path:="$ROSBAG_PATH" config_path:=$CONFIG_PATH
        mv "$VINS_OUTPUT_PATH"/* "$OUTPUT"
        mv ~/.ros/pose_graph.txt "$OUTPUT"
        mv ~/.ros/VINS_KeyframeMemUsageKB.txt "$OUTPUT"
        mv ~/.ros/VINS_KeyframeTrackTiming.txt "$OUTPUT"
    done
done
