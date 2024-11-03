#!/bin/bash

set -e

for i in {0..17}; do
    rosbag_idx=$(printf "%02d" "$i")
    rosbag_name="spagna_train0_$rosbag_idx"
    python3 process_openvins.py "/user/openvins-$rosbag_name.txt" "/user/openvins-$rosbag_name.tum"
done

# for i in {0..17}; do
#     rosbag_idx=$(printf "%02d" "$i")
#     rosbag_name="spagna_train0_$rosbag_idx"

#     rosbag_path="/mnt/ssd_4T/tianyi_data/vbr/vbr_slam/spagna/spagna_train0/spagna_train0_$rosbag_idx.bag"
#     ls "$rosbag_path"

#     roslaunch hl_orbslam3_wrapper openvins.launch bag:=$rosbag_path

#     mv /tmp/traj_estimate.txt "/user/openvins-$rosbag_name.txt"
# done




# python3 combine_rosbag.py

# roslaunch hl_orbslam3_wrapper openvins.launch bag:=/mnt/ssd_4T/tianyi_data/vbr/vbr_slam/diag/diag_train0/diag_train0.bag
# python3 process_openvins.py /tmp/traj_estimate.txt /user/openvins-diag_train0.tum

# roslaunch hl_orbslam3_wrapper openvins.launch bag:=/mnt/ssd_4T/tianyi_data/vbr/vbr_slam/pincio/pincio_train0/pincio_train0.bag
# python3 process_openvins.py /tmp/traj_estimate.txt /user/openvins-pincio_train0.tum

# roslaunch hl_orbslam3_wrapper openvins.launch bag:=/mnt/ssd_4T/tianyi_data/vbr/vbr_slam/ciampino/ciampino_train0/ciampino_train0.bag
# python3 process_openvins.py /tmp/traj_estimate.txt /user/openvins-ciampino_train0.tum

# roslaunch hl_orbslam3_wrapper openvins.launch bag:=/mnt/ssd_4T/tianyi_data/vbr/vbr_slam/ciampino/ciampino_train1/ciampino_train1.bag
# python3 process_openvins.py /tmp/traj_estimate.txt /user/openvins-ciampino_train1.tum

# roslaunch hl_orbslam3_wrapper openvins.launch bag:=/mnt/ssd_4T/tianyi_data/vbr/vbr_slam/campus/campus_train0/campus_train0.bag
# python3 process_openvins.py /tmp/traj_estimate.txt /user/openvins-campus_train0.tum

# roslaunch hl_orbslam3_wrapper openvins.launch bag:=/mnt/ssd_4T/tianyi_data/vbr/vbr_slam/campus/campus_train1/campus_train1.bag
# python3 process_openvins.py /tmp/traj_estimate.txt /user/openvins-campus_train1.tum

# roslaunch hl_orbslam3_wrapper openvins.launch bag:=/mnt/ssd_4T/tianyi_data/vbr/vbr_slam/colosseo/colosseo_train0/colosseo_train0.bag
# python3 process_openvins.py /tmp/traj_estimate.txt /user/openvins-colosseo_train0.tum`