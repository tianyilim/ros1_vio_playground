#!/bin/bash

set -e

# Need to make distinction between vbr-AD (20hz lidar/cams) and vbr-handheld (30hz lidar/cams)
# So need to run Kimera-VIO with different rate_hz for different environments

BASE_PATH="/mnt/ssd_4T/tianyi_data/vbr/vbr_slam"
KIMERA_CONFIG_PATH="/catkin_ws/src/hl_orbslam3_wrapper/cfg/kimera/params/vbr"

AD_ENVIRONMENTS=("campus" "ciampino")                 # runs at 20 hz
HH_ENVIRONMENTS=("colosseo" "diag" "pincio" "spagna") # runs at 30 hz

# Function to find and replace the line with the first substring in a file
find_and_replace_in_file() {
    local file="$1"
    local search="$2"
    local replace="$3"
    sed -i "s/$search.*/$replace/" "$file"
}

modify_rate_hz_in_configs() {
    target_rate="$1"
    files_with_rate=("RightCameraParams.yaml" "LeftCameraParams.yaml" "ExternalOdometryParams.yaml")

    for file in "${files_with_rate[@]}"; do
        find_and_replace_in_file "$KIMERA_CONFIG_PATH/$file" "rate_hz" "rate_hz: $target_rate"
    done
}

# Actual logic to run Kimera and save outputs
run_kimera_on_seqs() {
    ENVIRONMENTS=("$@")
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
            rm -rf /catkin_ws/src/Kimera-VIO-ROS/output_logs/vbr_"$subfolder_name"
            mkdir -p /catkin_ws/src/Kimera-VIO-ROS/output_logs/vbr_"$subfolder_name"
            cp -r /catkin_ws/src/Kimera-VIO-ROS/output_logs/vbr/* /catkin_ws/src/Kimera-VIO-ROS/output_logs/vbr_"$subfolder_name"
        done
    done
}

# run stuff
modify_rate_hz_in_configs 30
run_kimera_on_seqs "${HH_ENVIRONMENTS[@]}"
modify_rate_hz_in_configs 20
run_kimera_on_seqs "${AD_ENVIRONMENTS[@]}"
