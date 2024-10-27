#!/bin/bash

ROSBAG_ROOT="/mnt/ssd_4T/tianyi_data/arche-long-indiv-bags"

CAPTURES=(
    "b3_to_b5_loop_round_field_traverse_2024-07-02_13.00.29_000.bag"
    "b5_to_gp_to_b3_to_b4_traverse_2024-07-04_09.10.58_000.bag"
    "b4_to_gp_to_d2_traverse_2024-07-04_09.39.45_000.bag"
    "d2_to_gp_to_b5_2024-07-04_08.51.46_000.bag"
    "b5_to_b3_big_traverse_2024-07-04_08.57.55_000.bag"
)

source /catkin_ws/devel/setup.bash

cd ../launch || exit

for capture in "${CAPTURES[@]}"; do
    echo "Processing capture: $capture"

    ls "$ROSBAG_ROOT/$capture"
    ROSBAG_PATH="$ROSBAG_ROOT/$capture"

    rm -f /user/orbslam3_traj.tum
    roslaunch arche-orbslam-mono.launch rosbag_path:=$ROSBAG_PATH viz:=False
    mv /user/orbslam3_traj.tum /user/orbslam3-$capture.tum
    mv ~/.ros/KeyframeMemUsageKB.txt /user/orbslam3-"$capture"_memUsageKB.txt
    mv ~/.ros/KeyframeTrackTiming.txt /user/orbslam3-"$capture"_timing.txt

    roslaunch arche-vins-mono.launch rosbag_path:=$ROSBAG_PATH viz:=False
    mv /user/vins-mono/output/vio.csv /user/vins_mono.tum
    mv ~/.ros/VINS_KeyframeMemUsageKB.txt /user/orbslam3-"$capture"_memUsageKB.txt
    mv ~/.ros/VINS_KeyframeTrackTiming.txt /user/orbslam3-"$capture"_timing.txt

done
