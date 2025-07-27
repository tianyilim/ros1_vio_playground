#!/bin/bash
set -e

VINS_CONFIG_PATH="/catkin_ws/src/VINS-Fusion/config/kitti_odom"
VINS_OUTPUT_PATH="/home/tony-ws1/output"

function move_vins_outputs() {
    local traj_name="kitti_$1"
    echo "Moving outputs for $traj_name"

    sudo mv $VINS_OUTPUT_PATH/vio.csv /user/vins-stereo_"$traj_name".tum
    # sudo mv /tmp/VINS_KeyframeMemUsageKB.txt /user/vins-stereo_"$traj_name"_KeyframeMemUsageKB.txt
    # sudo mv /tmp/VINS_KeyframeTrackTiming.txt /user/vins-stereo_"$traj_name"_KeyframeTrackTiming.txt
}

mkdir -p /user/vins-kitti
mkdir -p $VINS_OUTPUT_PATH

for i in {00..02}; do
    echo "Processing $i"
    rosrun vins kitti_odom_test "$VINS_CONFIG_PATH"/kitti_config00-02.yaml \
        /mnt/ssd_2T/kitti/dataset/sequences/"$i"
    move_vins_outputs $i
done

echo "Processing 03"
rosrun vins kitti_odom_test $VINS_CONFIG_PATH/kitti_config03.yaml \
    /mnt/ssd_2T/kitti/dataset/sequences/03
move_vins_outputs 03

# for i in {04..12}; do
for i in {04..10}; do
    echo "Processing $i"
    rosrun vins kitti_odom_test $VINS_CONFIG_PATH/kitti_config04-12.yaml \
        /mnt/ssd_2T/kitti/dataset/sequences/"$i"
    move_vins_outputs $i
done

# for i in {13..21}; do
#     echo "Processing $i"
#     # ./stereo_kitti ../../Vocabulary/ORBvoc.txt KITTI00-02.yaml /mnt/ssd_2T/kitti/dataset/sequences/"$i" 0
#     # move_orbslam_outputs $i

#     rosrun vins kitti_odom_test $VINS_CONFIG_PATH/kitti_config13-21.yaml \
#         /mnt/ssd_2T/kitti/dataset/sequences/"$i"
#     move_vins_outputs $i
# done
