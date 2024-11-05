#!/usr/bin/env python

import argparse
from pathlib import Path

import rospy
import cv_bridge
from sensor_msgs.msg import Image
import cv2


class ImageDownsampler:
    def __init__(self, topic: str) -> None:
        rospy.loginfo(f"Input topic: {topic}")

        self.out_topic = topic + "_downsampled"
        self.num_poses_written = 0

        rospy.loginfo("Started image downsampler node.")

        # Initialize the ROS node
        rospy.init_node('odometry_to_tum', anonymous=True)

        self.bridge = cv_bridge.CvBridge()

        # Create downsample publisher
        self.down_img_pub = rospy.Publisher(self.out_topic, Image, queue_size=10)

        # Subscribe to the /odom topic
        rospy.Subscriber(args.topic, Image, self.odometry_callback)

    def odometry_callback(self, msg: Image):
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        cv_img = cv2.resize(cv_img, (0, 0), fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
        new_msg = self.bridge.cv2_to_imgmsg(cv_img, encoding='passthrough')
        new_msg.header = msg.header

        self.down_img_pub.publish(new_msg)


def main(args):

    print(args)
    yomomma = ImageDownsampler(args.topic)

    # Keep the node running
    while not rospy.is_shutdown():
        rospy.spin()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="ROS node to downsample incoming image by /2")
    parser.add_argument('topic', type=str, help="ROS Image topic to subscribe to")
    args, _ = parser.parse_known_args()

    try:
        main(args)
    except rospy.ROSInterruptException:
        pass
