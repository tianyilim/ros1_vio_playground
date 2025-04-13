#!/bin/bash

set -e

# Run KISS-ICP on all VBR datasets
# Need to also save path to file

# List all train rosbags we have

BASE_PATH="/mnt/ssd_4T/tianyi_data/vbr/vbr_slam"
ENVIRONMENTS_HANDHELD=(
    "colosseo"
    "diag"
    "pincio"
    "spagna"
)
ENVIRONMENTS_DRIVING=(
    "campus"
    "ciampino"
)

for ENV in "${ENVIRONMENTS_HANDHELD[@]}"; do
    # Iterate through subfolders
    for SUBFOLDER_PARENT in "$BASE_PATH"/"$ENV"/*; do
        SUBFOLDER=$(basename "$SUBFOLDER_PARENT")
        echo "Iterating through $ENV/$SUBFOLDER"

        ROSBAG_PATH=$BASE_PATH/$ENV/$SUBFOLDER/$SUBFOLDER.bag

        roslaunch hl_orbslam3_wrapper vbr-orbslam-stereo.launch rosbag_path:="$ROSBAG_PATH" \
            settings_filepath:="/catkin_ws/src/hl_orbslam3_wrapper/cfg/vbr-orbslam-stereo-handheld.yaml"
        mv /user/orbslam3_traj.tum /user/orbslam3-"$SUBFOLDER".tum
    done
done

for ENV in "${ENVIRONMENTS_DRIVING[@]}"; do
    # Iterate through subfolders
    for SUBFOLDER_PARENT in "$BASE_PATH"/"$ENV"/*; do
        SUBFOLDER=$(basename "$SUBFOLDER_PARENT")
        echo "Iterating through $ENV/$SUBFOLDER"

        ROSBAG_PATH=$BASE_PATH/$ENV/$SUBFOLDER/$SUBFOLDER.bag

        roslaunch hl_orbslam3_wrapper vbr-orbslam-stereo.launch rosbag_path:="$ROSBAG_PATH" \
            settings_filepath:="/catkin_ws/src/hl_orbslam3_wrapper/cfg/vbr-orbslam-stereo-driving.yaml"
        mv /user/orbslam3_traj.tum /user/orbslam3-"$SUBFOLDER".tum
    done
done
