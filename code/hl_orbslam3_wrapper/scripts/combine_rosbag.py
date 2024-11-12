"""
This scripts combines the multiple smaller rosbags in the VBR dataset into a single rosbag.
"""

from pathlib import Path
from typing import NamedTuple, Optional

import cv2
import natsort
import pytransform3d.transformations as pt
import rosbag
import rospy
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry
from pytransform3d.transform_manager import (NumpyTimeseriesTransform,
                                             TemporalTransformManager)
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from tqdm import tqdm

TO_DEBAYER = False
CAM_TO_SYNC_TO = "/camera_left/image_raw"
ODOM_FRAME = "odom"
ODOM_CHILD_FRAME = "os_sensor"    # if using kiss_icp, this is the frame_id of the sensor
ODOM_TOPIC_NAME = "/odometry"


class TransformPQ(NamedTuple):
    '''(tx, ty, tz, qw, qx, qy, qz)'''
    tx: float
    ty: float
    tz: float
    qw: float
    qx: float
    qy: float
    qz: float


def read_odom_file(input_tum: Path):
    '''Extract odom data from an input TUM file'''
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
        pqs.append(TransformPQ(x, y, z, qw, qx, qy, qz))

    tm = TemporalTransformManager()
    tm.add_transform("rig", "world", NumpyTimeseriesTransform(times, pqs))

    return tm, times


def _unix_to_ros_time(stamp: int) -> rospy.Time:
    """
    Converts from unix timestamp (int) to rospy.Time()

    :param int stamp: unix timestamp
    :return: ros timestamp
    """
    secs = stamp // 1000_000_000
    nsecs = stamp % 1000_000_000
    return rospy.Time(secs, nsecs)


def _create_ros_header(timestamp: int, frame_id: str, seq: int) -> Header:
    header = Header()
    header.stamp = _unix_to_ros_time(timestamp)
    header.frame_id = frame_id
    header.seq = seq

    return header


def _create_odometry_msg(pq: TransformPQ, sensor_frame_id: str, header: Header) -> Odometry:
    """Creates odometry message with the given pose and header.

    Args:
        pq (TransformPQ): T_w_c read from trajectories.txt
        sensor_frame_id (str): name of sensor TF frame
        header (Header): header. NOTE: This should have a frame_id of Odometry frame, not sensor frame.

    Returns:
        nav_msgs.Odometry
    """
    msg = Odometry()

    msg.header = header
    msg.child_frame_id = sensor_frame_id

    msg.pose.pose.position.x = pq.tx
    msg.pose.pose.position.y = pq.ty
    msg.pose.pose.position.z = pq.tz

    msg.pose.pose.orientation.w = pq.qw
    msg.pose.pose.orientation.x = pq.qx
    msg.pose.pose.orientation.y = pq.qy
    msg.pose.pose.orientation.z = pq.qz

    # We don't set the covariance in this case.

    # We don't set the twist in this case as it's not available in the Capture format.

    return msg


def combine_rosbag(inbags, outbag, odom_file: Optional[Path] = None):
    bridge = CvBridge()

    if odom_file is not None:
        tm, odom_times = read_odom_file(odom_file)

    outbag = rosbag.Bag(outbag, "w")
    print(f"Merging {len(inbags)} input rosbags...")
    with outbag as outbag:
        topic: str
        for bag_number, inbag in enumerate(tqdm(inbags)):
            print(f"Processing bag {bag_number + 1}/{len(inbags)}")
            try:
                inbag = rosbag.Bag(inbag, "r")
            except BaseException as e:
                print(f"Failed to open bag: {e}")
                break

            for topic, msg, t in tqdm(inbag.read_messages(), total=inbag.get_message_count()):  # type: ignore
                # Write all other messages except PC2
                if topic == "/ouster/imu" or topic == "/ouster/points":
                    continue

                # If it's an image file, debayer and convert to MONO8
                if TO_DEBAYER and "camera" in topic:
                    try:
                        cv_image_bayer = bridge.imgmsg_to_cv2(msg, desired_encoding="bayer_rggb8")
                        # print(cv_image_bayer.dtype, cv_image_bayer.shape)
                        # Convert Bayer image to grayscale (mono8)
                        cv_image_mono8 = cv2.cvtColor(cv_image_bayer, cv2.COLOR_BAYER_RG2RGB)
                        # print(cv_image_mono8.dtype, cv_image_mono8.shape)
                        # print('--' * 40)
                        mono8_msg: Image = bridge.cv2_to_imgmsg(cv_image_mono8, encoding="rgb8")
                    except CvBridgeError as e:
                        print(e)
                        continue

                    mono8_msg.header = msg.header
                    msg = mono8_msg

                # Synchronize odom to image topic
                if odom_file is not None and topic == CAM_TO_SYNC_TO:
                    img_time = msg.header.stamp.to_sec()
                    if img_time <= odom_times[-1] and img_time >= odom_times[0]:
                        transform = tm.get_transform_at_time("rig", "world", img_time)
                        transform_pq = TransformPQ(*pt.pq_from_transform(transform))
                        header = _create_ros_header(int(img_time * 1e9), ODOM_FRAME, msg.header.seq)
                        odom_msg = _create_odometry_msg(transform_pq, ODOM_CHILD_FRAME, header)
                        outbag.write(ODOM_TOPIC_NAME, odom_msg, t)

                outbag.write(topic, msg, t)


if __name__ == "__main__":

    BASE_PATH = Path("/mnt/ssd_4T/tianyi_data/vbr/vbr_slam")
    ENVIRONMENTS = [
        "campus",
        "ciampino",
        "colosseo",
        "diag",
        "pincio",
        "spagna"]

    for env in ENVIRONMENTS:
        for subfolder in (BASE_PATH / env).iterdir():

            if "train" in subfolder.name:
                # print(f"Skipping subfolder {subfolder.name} with 'train' data")
                continue

            # All files in subfolder that end in .bag
            print(subfolder)
            bag_files = list(subfolder.glob(f"{subfolder.name}_*.bag"))
            bag_files = natsort.natsorted(bag_files)
            print(bag_files)

            odom_file = subfolder / f"{subfolder.name}_kiss_icp.txt"
            assert odom_file.exists()

            outbag = subfolder / f"{subfolder.name}.bag"
            inbags = [str(bag_file) for bag_file in bag_files]

            # if outbag.exists():
            #     print(f"Output bag {outbag} already exists, skipping...")
            #     continue

            combine_rosbag(inbags, outbag, odom_file)

            print('+========+' * 10)

    exit(0)
