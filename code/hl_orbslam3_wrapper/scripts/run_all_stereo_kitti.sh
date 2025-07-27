#!/bin/bash

cd /ORB_SLAM3/Examples/Stereo

function move_orbslam_outputs() {
    local traj_name="kitti_$1"
    echo "Moving outputs for $traj_name"
    mv CameraTrajectory.txt orbslam3_"$traj_name".kitti
    mv KeyframeMemUsageKB.txt orbslam3_"$traj_name"_KeyframeMemUsageKB.txt
    mv KeyframeTrackTiming.txt orbslam3_"$traj_name"_KeyframeTrackTiming.txt
}

for i in {00..02}; do
    echo "Processing $i"
    ./stereo_kitti ../../Vocabulary/ORBvoc.txt KITTI00-02.yaml /mnt/ssd_2T/kitti/dataset/sequences/"$i" 0
    move_orbslam_outputs $i
done

echo "Processing $i"
./stereo_kitti ../../Vocabulary/ORBvoc.txt KITTI03.yaml /mnt/ssd_2T/kitti/dataset/sequences/03 0
move_orbslam_outputs 03

for i in {04..12}; do
    echo "Processing $i"
    ./stereo_kitti ../../Vocabulary/ORBvoc.txt KITTI04-12.yaml /mnt/ssd_2T/kitti/dataset/sequences/"$i" 0
    move_orbslam_outputs $i
done

for i in {13..21}; do
    echo "Processing $i"
    ./stereo_kitti ../../Vocabulary/ORBvoc.txt KITTI00-02.yaml /mnt/ssd_2T/kitti/dataset/sequences/"$i" 0
    move_orbslam_outputs $i
done
