```bash
rosbag play hl_stereo_test_imu.bag --clock  --wait-for-subscribers \
    --topics /imu/gyro /imu/accel /hetrf/image_rect /hetlf/image_rect
```
