
from pathlib import Path
from typing import List, NamedTuple, Optional

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


def combine_rosbag(inbags_paths: List[str], outbag_paths: str):

    outbag = rosbag.Bag(outbag_paths, "w")
    print(f"Merging {len(inbags_paths)} input rosbags...")
    with outbag as outbag:
        topic: str
        for bag_number, inbag in enumerate(tqdm(inbags_paths)):
            print(f"Processing bag {bag_number + 1}/{len(inbags_paths)}")
            try:
                inbag = rosbag.Bag(inbag, "r")
            except BaseException as e:
                print(f"Failed to open bag: {e}")
                break

            for topic, msg, t in tqdm(inbag.read_messages(), total=inbag.get_message_count()):  # type: ignore
                # Write only PC2
                if topic != "/ouster/points":
                    continue

                outbag.write(topic, msg, t)


if __name__ == "__main__":

    BASE_PATH = Path("/mnt/ssd_4T/tianyi_data/vbr/vbr_slam")
    ENVIRONMENTS = ["spagna"]

    for env in ENVIRONMENTS:
        for subfolder in (BASE_PATH / env).iterdir():

            if "test" in subfolder.name:
                continue

            # All files in subfolder that end in .bag
            print("Combining lidar rosbags in", subfolder)
            print(subfolder)
            bag_files = list(subfolder.glob(f"{subfolder.name}_*.bag"))
            bag_files = natsort.natsorted(bag_files)
            print(bag_files)

            outbag = str(subfolder / f"{subfolder.name}_lidaronly.bag")
            inbags = [str(bag_file) for bag_file in bag_files]

            # if outbag.exists():
            #     print(f"Output bag {outbag} already exists, skipping...")
            #     continue

            combine_rosbag(inbags, outbag)

            print('+========+' * 10)

    exit(0)
