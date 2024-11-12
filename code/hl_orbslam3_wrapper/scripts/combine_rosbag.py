from pathlib import Path

import cv2
import natsort
import rosbag
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from tqdm import tqdm


def combine_rosbag(inbags, outbag):
    bridge = CvBridge()

    outbag = rosbag.Bag(outbag, "w")
    print(f"Merging {len(inbags)} input rosbags...")
    with outbag as outbag:
        topic: str
        for bag_number, inbag in enumerate(tqdm(inbags)):
            print(f"Processing bag {bag_number + 1}/{len(inbags)}")
            try:
                inbag = rosbag.Bag(inbag, "r")
            except BaseException:
                break

            for topic, msg, t in tqdm(inbag.read_messages(), total=inbag.get_message_count()):  # type: ignore
                # Write all other messages except PC2
                if topic == "/ouster/imu" or topic == "/ouster/points":
                    continue

                # If it's an image file, debayer and convert to MONO8
                if "camera" in topic:
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

                outbag.write(topic, msg, t)


if __name__ == "__main__":

    BASE_PATH = Path("/mnt/ssd_4T/tianyi_data/vbr/vbr_slam")
    ENVIRONMENTS = ["campus",
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
            bag_files = list(subfolder.glob("*.bag"))
            bag_files = natsort.natsorted(bag_files)
            print(bag_files)

            outbag = subfolder / f"{subfolder.name}.bag"
            inbags = [str(bag_file) for bag_file in bag_files]
            combine_rosbag(inbags, outbag)

            print('+========+' * 10)

    exit(0)
