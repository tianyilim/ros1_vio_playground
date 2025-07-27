#!/bin/bash

# set -e

# Run KISS-ICP on all VBR datasets
# Need to also save path to file

# List all train rosbags we have

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

for ENV in "${ENVIRONMENTS[@]}"; do
    # Iterate through subfolders
    for SUBFOLDER in $(ls $BASE_PATH/$ENV); do
        echo "Iterating through $ENV/$SUBFOLDER"

        ROSBAG_PATH=$BASE_PATH/$ENV/$SUBFOLDER/$SUBFOLDER.bag

        roslaunch hl_orbslam3_wrapper vbr-vins-stereo.launch viz:=False rosbag_path:="$ROSBAG_PATH" config_path:=$CONFIG_PATH
        mv /user/vins-stereo/output/vio.csv /user/vins-stereo-"$SUBFOLDER".tum
        mv ~/.ros/VINS_KeyframeMemUsageKB.txt /user/vins-stereo-"$SUBFOLDER"_memUsageKB.txt
        mv ~/.ros/VINS_KeyframeTrackTiming.txt /user/vins-stereo-"$SUBFOLDER"_timing.txt
    done
done
