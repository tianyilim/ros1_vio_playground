#!/bin/bash
set -e
cd /ORB_SLAM3/Examples/Stereo

VINS_CONFIG_PATH="/catkin_ws/src/VINS-Fusion/config/kitti_odom"
VINS_OUTPUT_PATH="/home/tony-ws1/output"

function move_vins_outputs() {
    local traj_name="kitti_$1"
    echo "Moving outputs for $traj_name"
    local OUTPUT_PATH=/user/vins-"$traj_name"
    mkdir -p "$OUTPUT_PATH"

    cp $VINS_OUTPUT_PATH/* "$OUTPUT_PATH"
    cp ~/.ros/pose_graph.txt "$OUTPUT_PATH"
    # sudo mv /tmp/VINS_KeyframeTrackTiming.txt /user/vins-stereo_"$traj_name"_KeyframeTrackTiming.txt
}

function run_vins() {
    roslaunch hl_orbslam3_wrapper kitti-vins-stereo-lc.launch config_path:="$1" seq_path:="$2"
}

mkdir -p $VINS_OUTPUT_PATH

for i in {00..02}; do
    echo "Processing $i"
    run_vins "$VINS_CONFIG_PATH"/kitti_config00-02.yaml \
        /mnt/ssd_2T/kitti/dataset/sequences/"$i"
    move_vins_outputs "$i"
done

echo "Processing 03"
run_vins $VINS_CONFIG_PATH/kitti_config03.yaml \
    /mnt/ssd_2T/kitti/dataset/sequences/03
move_vins_outputs 03

for i in {04..10}; do
    echo "Processing $i"
    run_vins $VINS_CONFIG_PATH/kitti_config04-12.yaml \
        /mnt/ssd_2T/kitti/dataset/sequences/"$i"
    move_vins_outputs "$i"
done
