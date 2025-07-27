#!/bin/bash

set -e

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

for ENV in "${ENVIRONMENTS[@]}"; do
    # Iterate through subfolders
    for SUBFOLDER in $(ls $BASE_PATH/$ENV); do
        echo "Iterating through $ENV/$SUBFOLDER"

        if [[ $SUBFOLDER == *train* ]]; then
            echo "Skipping train folder $SUBFOLDER for now."
            continue
        fi

        # Add all rosbags of interest to a space separated string
        rosbags_of_interest=$(ls "$BASE_PATH/$ENV/$SUBFOLDER/$SUBFOLDER"_*.bag)
        # This is needed because ls delimiter is a newline
        rosbags_of_interest=$(echo $rosbags_of_interest | tr '\n' ' ')

        out_file="$BASE_PATH/$ENV/$SUBFOLDER/$SUBFOLDER"_kiss_icp.txt

        echo Writing to "$out_file"
        echo $rosbags_of_interest

        roslaunch hl_orbslam3_wrapper kiss_icp.launch \
            bagfile:="$rosbags_of_interest" \
            topic:=/ouster/points \
            visualize:=False \
            pose_out_file:="$out_file"
    done
done
