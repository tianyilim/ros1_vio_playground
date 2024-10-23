#!/bin/bash

ROSBAG_PATH="/mnt/ssd_2T/hilti-22/exp23_the_sheldonian_slam_part_0+.bag"
source /catkin_ws/devel/setup.bash

cd ../launch || exit

rm -f /user/orbslam3_traj.tum
roslaunch hilti22-orbslam-mono.launch rosbag_path:=$ROSBAG_PATH viz:=False && mv /user/orbslam3_traj.tum /user/orbslam3_mono.tum

rm -f /user/orbslam3_traj.tum
roslaunch hilti22-orbslam-stereo.launch rosbag_path:=$ROSBAG_PATH viz:=False && mv /user/orbslam3_traj.tum /user/orbslam3_stereo.tum

roslaunch hilti22-vins-mono.launch rosbag_path:=$ROSBAG_PATH viz:=False && mv /user/vins-mono/output/vio.csv /user/vins_mono.tum

roslaunch hilti22-vins-stereo.launch rosbag_path:=$ROSBAG_PATH viz:=False && mv /user/vins-stereo/output/vio.csv /user/vins_stereo.tum
