# ROS wrapper to run V-SLAM algos

Look at the files in the `launch` folder. It should give sufficient information on what is expected there.

This is meant to be run in the Docker container provided in this repo.

## ORB-SLAM3

- 

## VINS-MONO

```bash
rosbag play hl_stereo_test_imu.bag --clock  --wait-for-subscribers \
    --topics /imu/gyro /imu/accel /hetrf/image_rect /hetlf/image_rect
```
