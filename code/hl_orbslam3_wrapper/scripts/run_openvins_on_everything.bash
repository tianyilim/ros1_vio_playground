#!/bin/bash

set -e

BASE_PATH="/mnt/ssd_4T/tianyi_data/vbr/vbr_slam"
ENVIRONMENTS_CAR=(
    "campus"
    "ciampino"
)
ENVIRONMENTS_HANDHELD=(
    "colosseo"
    "diag"
    "pincio"
    "spagna"
)

for ENV in "${ENVIRONMENTS_CAR[@]}"; do
    # Iterate through subfolders
    for SUBFOLDER in $(ls $BASE_PATH/$ENV); do
        echo "Iterating through Car Env $ENV/$SUBFOLDER"
        ROSBAG_PATH="$BASE_PATH/$ENV/$SUBFOLDER/$SUBFOLDER".bag

        roslaunch hl_orbslam3_wrapper openvins.launch bag:=$ROSBAG_PATH config:=vbr_car
        mv /tmp/traj_estimate.txt "/user/openvins-$SUBFOLDER.txt" &&
            python3 process_openvins.py "/user/openvins-$SUBFOLDER.txt" "/user/openvins-$SUBFOLDER.tum"
    done
done

for ENV in "${ENVIRONMENTS_HANDHELD[@]}"; do
    # Iterate through subfolders
    for SUBFOLDER in $(ls $BASE_PATH/$ENV); do
        echo "Iterating through Handheld Env $ENV/$SUBFOLDER"
        ROSBAG_PATH="$BASE_PATH/$ENV/$SUBFOLDER/$SUBFOLDER".bag

        roslaunch hl_orbslam3_wrapper openvins.launch bag:=$ROSBAG_PATH config:=vbr
        mv /tmp/traj_estimate.txt "/user/openvins-$SUBFOLDER.txt" &&
            python3 process_openvins.py "/user/openvins-$SUBFOLDER.txt" "/user/openvins-$SUBFOLDER.tum"
    done
done
