import rosbag
from tqdm import tqdm
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import cv2


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
                        cv_image_mono8 = cv2.cvtColor(cv_image_bayer, cv2.COLOR_BAYER_RG2GRAY)
                        # print(cv_image_mono8.dtype, cv_image_mono8.shape)
                        # print('--' * 40)
                        mono8_msg: Image = bridge.cv2_to_imgmsg(cv_image_mono8, encoding="mono8")
                    except CvBridgeError as e:
                        print(e)
                        continue

                    mono8_msg.header = msg.header
                    msg = mono8_msg

                outbag.write(topic, msg, t)

# outbag = "/mnt/ssd_4T/tianyi_data/vbr/vbr_slam/diag/diag_train0/diag_train0.bag"
# inbags = [
#     f"/mnt/ssd_4T/tianyi_data/vbr/vbr_slam/diag/diag_train0/diag_train0_{i:02d}.bag" for i in range(14)
# ]
# combine_rosbag(inbags, outbag)

# outbag = "/mnt/ssd_4T/tianyi_data/vbr/vbr_slam/pincio/pincio_train0/pincio_train0.bag"
# inbags = [
#     f"/mnt/ssd_4T/tianyi_data/vbr/vbr_slam/pincio/pincio_train0/pincio_train0_{i:02d}.bag" for i in range(18)
# ]
# combine_rosbag(inbags, outbag)

# outbag = "/mnt/ssd_4T/tianyi_data/vbr/vbr_slam/ciampino/ciampino_train0/ciampino_train0.bag"
# inbags = [
#     f"/mnt/ssd_4T/tianyi_data/vbr/vbr_slam/ciampino/ciampino_train0/ciampino_train0_{i:02d}.bag" for i in range(12)
# ]
# combine_rosbag(inbags, outbag)

# outbag = "/mnt/ssd_4T/tianyi_data/vbr/vbr_slam/ciampino/ciampino_train1/ciampino_train1.bag"
# inbags = [
#     f"/mnt/ssd_4T/tianyi_data/vbr/vbr_slam/ciampino/ciampino_train1/ciampino_train1_{i:02d}.bag" for i in range(7)
# ]
# combine_rosbag(inbags, outbag)

# outbag = "/mnt/ssd_4T/tianyi_data/vbr/vbr_slam/campus/campus_train0/campus_train0.bag"
# inbags = [
#     f"/mnt/ssd_4T/tianyi_data/vbr/vbr_slam/campus/campus_train0/campus_train0_{i:02d}.bag" for i in range(5)
# ]
# combine_rosbag(inbags, outbag)

# outbag = "/mnt/ssd_4T/tianyi_data/vbr/vbr_slam/campus/campus_train1/campus_train1.bag"
# inbags = [
#     f"/mnt/ssd_4T/tianyi_data/vbr/vbr_slam/campus/campus_train1/campus_train1_{i:02d}.bag" for i in range(5)
# ]
# combine_rosbag(inbags, outbag)

# outbag = "/mnt/ssd_4T/tianyi_data/vbr/vbr_slam/colosseo/colosseo_train0/colosseo_train0.bag"
# inbags = [
#     f"/mnt/ssd_4T/tianyi_data/vbr/vbr_slam/colosseo/colosseo_train0/colosseo_train0_{i:02d}.bag" for i in range(12)
# ]
# combine_rosbag(inbags, outbag)


if __name__ == "__main__":
    outbag = "/mnt/ssd_4T/tianyi_data/vbr/vbr_slam/spagna/spagna_train0/spagna_train00-17.bag"
    inbags = [
        f"/mnt/ssd_4T/tianyi_data/vbr/vbr_slam/spagna/spagna_train0/spagna_train0_{i:02d}.bag" for i in range(18)
    ]
    combine_rosbag(inbags, outbag)
