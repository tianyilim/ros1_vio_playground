'''
The HILTI-22 dataset is in a rosbag. We convert it into a Capture file for our use.

Calibration can be extracted from https://storage.googleapis.com/hsc2022/calibration/2022322_calibration_files.zip
'''

import argparse
import typing as t
from pathlib import Path

import cv2
import natsort
import numpy as np
import pytransform3d.transformations as pt
import rosbag
from cv_bridge import CvBridge, CvBridgeError
from numpy.typing import NDArray
from pytransform3d.transform_manager import (NumpyTimeseriesTransform,
                                             TemporalTransformManager)
from tqdm import tqdm

PARSE_IMAGES = False
'''Set this True to parse stuff from input rosbag, this will take a long time'''

# Calibration taken from HILTI
T_cam0_imu = np.array([[0.006708021451800439, 0.9999264200621176, -0.01010727015365992, -0.04586422589354697],
                       [0.002425643641803532, 0.010091197021688258, 0.9999461405473755, 0.012631813183337478],
                       [0.9999745590269407, -0.00673217679702115, -0.0023577731969991467, -0.05098782892861867],
                       [0.0, 0.0, 0.0, 1.0]])
T_cam1_imu = np.array([[0.0016556126470597954, 0.9999840642813064, -0.005397233569448934, 0.06262779955244471],
                      [0.0009350089535379302, 0.005395690615039778, 0.9999850060281122, 0.01343471252962214],
                       [0.9999981923508761, -0.0016606342845620903, -0.0009260608798763448, -0.05050835847481363],
                       [0.0, 0.0, 0.0, 1.0]])
T_cam2_imu = np.array([[0.9999897552434941, 0.0042340516721277015, -0.0016006918802386892, -0.0068618144566339415],
                       [0.004238470700886935, -0.9999871881804255, 0.0027674611333545654, -0.007931259020147333],
                       [-0.0015889537990238962, -0.0027742172670246527, -0.9999948894591316, -0.034124943940492],
                       [0.0, 0.0, 0.0, 1.0]])
T_cam3_imu = np.array([[-0.9998916894135631, 0.007396041673023157, 0.01272430781071447, -0.0030245656200529533],
                       [0.01270714046408309, -0.002363530935315569, 0.9999164676625464, 0.011180668261220912],
                       [0.007425498159515405, 0.9999698556902047, 0.002269292399476633, -0.05718309342102409],
                       [0.0, 0.0, 0.0, 1.0]])
T_cam4_imu = np.array([[0.9999880402484476, -0.000942427662931895, -0.004799082221662863, 0.006416247252956556],
                      [0.004797008865106434, -0.0021900897852221157, 0.9999860960096802, 0.01670064540948574],
                       [-0.0009529249803788974, -0.9999971576643774, -0.0021855427584387965, -0.07488037729385075],
                       [0.0, 0.0, 0.0, 1.0]])
T_cams_imu = [T_cam0_imu, T_cam1_imu, T_cam2_imu, T_cam3_imu, T_cam4_imu]

# width, height, fx, fy, cx, cy
cam0_intrinsics = [720, 540, 351.31400364193297, 351.4911744656785, 367.8522793375995, 253.8402144980996]
cam1_intrinsics = [720, 540, 352.6489794433894, 352.8586498571586, 347.8170010310082, 270.5806692485468]
cam2_intrinsics = [720, 540, 350.70040966794545, 350.8792449525716, 375.2977403521422, 268.5927747079796]
cam3_intrinsics = [720, 540, 352.9514843860555, 353.32837903547403, 363.93345228274336, 266.14511705007413]
cam4_intrinsics = [720, 540, 351.5132148653381, 351.7557554938886, 342.8425988673232, 259.91793254535776]
cams_intrinsics = [cam0_intrinsics, cam1_intrinsics, cam2_intrinsics, cam3_intrinsics, cam4_intrinsics]
cams_intrinsics = [[str(x) for x in cam_int] for cam_int in cams_intrinsics]


def get_cam_topic_by_idx(idx: int) -> str:
    return f"/alphasense/cam{idx}/image_rect"


def get_cam_idx_by_topic(topic: str) -> int:
    return int(topic.split("/")[2][-1])


def read_odom_file(input_tum: Path):
    times = []
    pqs = []

    with open(input_tum, 'r') as f:
        tum_lines = f.readlines()

    for line in tum_lines:
        line = line.strip()
        if line.startswith("#"):
            continue
        line = line.split()

        timestamp_s, x, y, z, qx, qy, qz, qw = map(float, line)

        times.append(timestamp_s)
        pqs.append((x, y, z, qw, qx, qy, qz))

    return times, pqs


def parse_odom(input_tum: Path, times_out_s: t.List[float]) -> t.List[t.Tuple[float, NDArray[np.float_]]]:
    """Odometry may not have the same timestamps as the images.
    This function interpolates the odometry to the image timestamps."""

    tm = TemporalTransformManager()
    times, pqs = read_odom_file(input_tum)

    tm.add_transform("rig", "world", NumpyTimeseriesTransform(times, pqs))

    times_out_s_filtered = [time for time in times_out_s if time >= times[0] and time <= times[-1]]
    # filter out duplicate timestamps
    times_out_s_filtered = natsort.natsorted(list(set(times_out_s_filtered)))

    # Interpolate times
    out: t.List[t.Tuple[float, NDArray[np.float_]]] = []
    for time in tqdm(times_out_s_filtered):
        T_w_rig_t = tm.get_transform_at_time("rig", "world", time)
        pq_w_rig_t = pt.pq_from_transform(T_w_rig_t)

        out.append((time, pq_w_rig_t))

    return out


def main(args):
    capture_path: Path = args.capture
    capture_raw_data_path = capture_path / "raw_data"
    capture_proc_path = capture_path / "proc"
    capture_proc_path.mkdir(parents=True, exist_ok=True)

    assert args.bag.exists(), f"Bag file {args.bag} does not exist"

    traj_times, _ = read_odom_file(args.traj)
    traj_start, traj_end = min(traj_times), max(traj_times)

    if PARSE_IMAGES:
        bridge = CvBridge()
        # Images
        # timestamp, sensor_id, image_path
        images_txt = []
        topics_to_read = [get_cam_topic_by_idx(i) for i in range(5)]
        with rosbag.Bag(args.bag) as bag:
            for topic, msg, t in tqdm(bag.read_messages(topics=topics_to_read),  # type: ignore
                                      total=bag.get_message_count()):
                cam_idx = get_cam_idx_by_topic(topic)

                # more convenient to save as us
                t_us: int = msg.header.stamp.to_nsec() // 1000

                # Skip those images that are not in trajectory
                if t_us < traj_start or t_us > traj_end:
                    continue

                try:
                    image_cv2 = bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
                except CvBridgeError as e:
                    print(e)
                    continue

                image_path = capture_raw_data_path / f"cam{cam_idx}" / f"{t_us}.png"
                image_path.parent.mkdir(parents=True, exist_ok=True)
                cv2.imwrite(str(image_path), image_cv2)

                images_txt.append((t_us, f"cam{cam_idx}", f"cam{cam_idx}/{t_us}.png"))

        # Write to images.txt, sorted by timestamp
        images_txt = sorted(images_txt, key=lambda x: x[0])
        with open(capture_path / "images.txt", 'w') as f:
            f.write("# timestamp, sensor_id, image_path\n")
            for images_txt_line in images_txt:
                f.write(f"{images_txt_line[0]}, {images_txt_line[1]}, {images_txt_line[2]}\n")
    else:
        # Read images.txt for timestamps
        print("Reading images.txt for timestamps")
        # Trajectories
        with open(capture_path / "images.txt", 'r') as f:
            f.readline()    # skip header
            images_lines = f.readlines()
            images_lines = [x.strip().split(",") for x in images_lines]

        image_lines_filt = []
        for line in images_lines:
            time_us = int(line[0])
            time_s = time_us / 1e6
            if time_s >= traj_start and time_s <= traj_end:
                image_lines_filt.append(line)

        # Write to images.txt, sorted by timestamp
        images_txt = sorted(image_lines_filt, key=lambda x: x[0])
        with open(capture_path / "images.txt", 'w') as f:
            f.write("# timestamp, sensor_id, image_path\n")
            for images_txt_line in images_txt:
                f.write(f"{images_txt_line[0]}, {images_txt_line[1]}, {images_txt_line[2]}\n")

    print("Writing rigs")
    # Rigs
    # rig_device_id, sensor_device_id, qw, qx, qy, qz, tx, ty, tz
    with open(capture_path / "rigs.txt", 'w') as f:
        f.write("# rig_device_id, sensor_device_id, qw, qx, qy, qz, tx, ty, tz\n")
        for i in range(5):
            T_imu_cam = np.linalg.inv(T_cams_imu[i])
            tx, ty, tz, qw, qx, qy, qz = pt.pq_from_transform(T_imu_cam)
            f.write(f"rig, cam{i}, {qw}, {qx}, {qy}, {qz}, {tx}, {ty}, {tz}\n")

    print("Writing sensors")
    # Sensors
    # sensor_id, name, sensor_type, [sensor_params]+
    with open(capture_path / "sensors.txt", 'w') as f:
        f.write("# sensor_id, name, sensor_type, [sensor_params]+\n")
        for i in range(5):
            f.write(f"cam{i}, cam{i}, camera, PINHOLE, {', '.join(cams_intrinsics[i])}\n")

    print("Reading images.txt for timestamps")
    # Trajectories
    with open(capture_path / "images.txt", 'r') as f:
        f.readline()    # skip header
        images_lines = f.readlines()
        images_lines = [x.strip().split(",") for x in images_lines]
        image_timestamps = [float(x[0]) / 1e6 for x in images_lines]

    print("Interpolating odom timestamps")
    # trajectories.txt
    # timestamp, device_id, qw, qx, qy, qz, tx, ty, tz, *covar
    interpolated_est_traj = parse_odom(args.traj, image_timestamps)
    with open(capture_path / "trajectories.txt", 'w') as f:
        f.write("# timestamp, device_id, qw, qx, qy, qz, tx, ty, tz, *covar\n")
        for time_s, pq in interpolated_est_traj:
            time_us = int(time_s * 1e6)
            tx, ty, tz, qw, qx, qy, qz = pq
            f.write(f"{time_us}, rig, {qw}, {qx}, {qy}, {qz}, {tx}, {ty}, {tz}\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", type=Path, help="Path to the rosbag")
    parser.add_argument("--traj", type=Path, help="Path to the trajectory file")
    parser.add_argument("--capture", type=Path, help="Path to Capture dataset root to be written")
    args = parser.parse_args()

    main(args)
