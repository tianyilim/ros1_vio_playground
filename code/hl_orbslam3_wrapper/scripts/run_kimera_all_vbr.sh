#!/bin/bash

set -e

# Need to make distinction between vbr-AD (20hz lidar/cams) and vbr-handheld (30hz lidar/cams)
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
    for SUBFOLDER in "$BASE_PATH/$ENV"/*; do
        echo "Iterating through $ENV/$SUBFOLDER"

        if [[ $SUBFOLDER == *test* ]]; then
            echo "Skipping test folder $SUBFOLDER for now."
            continue
        fi

        subfolder_name=$(basename "$SUBFOLDER")

        rosbag_path="$SUBFOLDER/$subfolder_name".bag
        echo "processing $rosbag_path"

        roslaunch hl_orbslam3_wrapper vbr-kimera-stereo.launch \
            bagfile:="$rosbag_path"

        # Copy output to a relevant location
        cp -r /catkin_ws/src/Kimera-VIO-ROS/output_logs/vbr /catkin_ws/src/Kimera-VIO-ROS/output_logs/vbr_"$subfolder_name"
    done
done
