#!/bin/bash

cd /ORB_SLAM3/Examples/Stereo

function move_orbslam_outputs() {
    local traj_name="kitti_$1"
    echo "Moving outputs for $traj_name"
    mv CameraTrajectory.txt orbslam3_"$traj_name".kitti
    mv KeyframeMemUsageKB.txt orbslam3_"$traj_name"_KeyframeMemUsageKB.txt
    mv KeyframeTrackTiming.txt orbslam3_"$traj_name"_KeyframeTrackTiming.txt
}

./stereo_kitti ../../Vocabulary/ORBvoc.txt KITTI00-02.yaml /mnt/ssd_2T/kitti/dataset/sequences/00 0
move_orbslam_outputs '00'
./stereo_kitti ../../Vocabulary/ORBvoc.txt KITTI00-02.yaml /mnt/ssd_2T/kitti/dataset/sequences/01 0
move_orbslam_outputs '01'
./stereo_kitti ../../Vocabulary/ORBvoc.txt KITTI00-02.yaml /mnt/ssd_2T/kitti/dataset/sequences/02 0
move_orbslam_outputs '02'
./stereo_kitti ../../Vocabulary/ORBvoc.txt KITTI03.yaml /mnt/ssd_2T/kitti/dataset/sequences/03 0
move_orbslam_outputs '03'
./stereo_kitti ../../Vocabulary/ORBvoc.txt KITTI04-12.yaml /mnt/ssd_2T/kitti/dataset/sequences/04 0
move_orbslam_outputs '04'
./stereo_kitti ../../Vocabulary/ORBvoc.txt KITTI04-12.yaml /mnt/ssd_2T/kitti/dataset/sequences/05 0
move_orbslam_outputs '05'
./stereo_kitti ../../Vocabulary/ORBvoc.txt KITTI04-12.yaml /mnt/ssd_2T/kitti/dataset/sequences/06 0
move_orbslam_outputs '06'
./stereo_kitti ../../Vocabulary/ORBvoc.txt KITTI04-12.yaml /mnt/ssd_2T/kitti/dataset/sequences/07 0
move_orbslam_outputs '07'
./stereo_kitti ../../Vocabulary/ORBvoc.txt KITTI04-12.yaml /mnt/ssd_2T/kitti/dataset/sequences/08 0
move_orbslam_outputs '08'
./stereo_kitti ../../Vocabulary/ORBvoc.txt KITTI04-12.yaml /mnt/ssd_2T/kitti/dataset/sequences/09 0
move_orbslam_outputs '09'
./stereo_kitti ../../Vocabulary/ORBvoc.txt KITTI04-12.yaml /mnt/ssd_2T/kitti/dataset/sequences/00 0
move_orbslam_outputs '00'
./stereo_kitti ../../Vocabulary/ORBvoc.txt KITTI04-12.yaml /mnt/ssd_2T/kitti/dataset/sequences/00 0
move_orbslam_outputs '00'
./stereo_kitti ../../Vocabulary/ORBvoc.txt KITTI04-12.yaml /mnt/ssd_2T/kitti/dataset/sequences/11 0
move_orbslam_outputs '11'
./stereo_kitti ../../Vocabulary/ORBvoc.txt KITTI04-12.yaml /mnt/ssd_2T/kitti/dataset/sequences/12 0
move_orbslam_outputs '12'
./stereo_kitti ../../Vocabulary/ORBvoc.txt KITTI00-02.yaml /mnt/ssd_2T/kitti/dataset/sequences/13 0
move_orbslam_outputs '13'
./stereo_kitti ../../Vocabulary/ORBvoc.txt KITTI00-02.yaml /mnt/ssd_2T/kitti/dataset/sequences/14 0
move_orbslam_outputs '14'
./stereo_kitti ../../Vocabulary/ORBvoc.txt KITTI00-02.yaml /mnt/ssd_2T/kitti/dataset/sequences/15 0
move_orbslam_outputs '15'
./stereo_kitti ../../Vocabulary/ORBvoc.txt KITTI00-02.yaml /mnt/ssd_2T/kitti/dataset/sequences/16 0
move_orbslam_outputs '16'
./stereo_kitti ../../Vocabulary/ORBvoc.txt KITTI00-02.yaml /mnt/ssd_2T/kitti/dataset/sequences/17 0
move_orbslam_outputs '17'
./stereo_kitti ../../Vocabulary/ORBvoc.txt KITTI00-02.yaml /mnt/ssd_2T/kitti/dataset/sequences/18 0
move_orbslam_outputs '18'
./stereo_kitti ../../Vocabulary/ORBvoc.txt KITTI00-02.yaml /mnt/ssd_2T/kitti/dataset/sequences/19 0
move_orbslam_outputs '19'
./stereo_kitti ../../Vocabulary/ORBvoc.txt KITTI00-02.yaml /mnt/ssd_2T/kitti/dataset/sequences/20 0
move_orbslam_outputs '20'
./stereo_kitti ../../Vocabulary/ORBvoc.txt KITTI00-02.yaml /mnt/ssd_2T/kitti/dataset/sequences/21 0
move_orbslam_outputs '21'