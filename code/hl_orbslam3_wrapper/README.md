# ROS wrapper to run V-SLAM algos

Look at the files in the `launch` folder. It should give sufficient information on what is expected there.

This is meant to be run in the Docker container provided in this repo.

## Rosbag examples
```bash
cd /Datasets/ssd/spot-rosbags
rosbag play --clock hydro/2023-11-24-10-53-HYDRO.bag \
    --pause \                                   # So that nodes have time to startup properly
    --start 32 \                                # For this rosbag, interesting stuff happens here
    /tf:=/tf_null /tf_static:=/tf_dev_null      # No TF shenanigans
```

## ORB-SLAM3

- ORB-SLAM3 was found to be brittle. You can still check the launchfiles, for example:
  - [Hololens Stereo-Inertial](./launch/hl_stereo_imu.launch)
  - [Spot Stereo-Inertial](./launch/hl_stereo_imu.launch)
- You'll also need to play a rosbag in a separate terminal:
  - `rosbag play --clock ...`
- Remember to set `rosparam set use_sim_time True`, so the nodes use the rosbag clock source.

## VINS-MONO
- [Spot Vins-Mono Onboard cam](./launch/spot_stereo_imu.launch)
- Launch RViz separately
   ```bash
   rviz -d viz/vinsmono_spot_bw.rviz
   ```

### Parsing VINS-MONO output
- You can save the Vins-Mono output following the instructions [here](https://github.com/HKUST-Aerial-Robotics/VINS-Mono?tab=readme-ov-file#3-visual-inertial-odometry-and-pose-graph-reuse-on-public-datasets).
- Essentially, enter `s` in the roslaunch terminal with `vins_estimator`. The current pose graph will be saved. By configuration, this is to `/user/vins_mono_out`.
- After which, you can run `code/hl_orbslam3_wrapper/scripts/process_vins_mono.py` to parse the output trajectory. Save it somewhere safe!
