#!/usr/bin/env python

import argparse
from pathlib import Path

import rospy
from nav_msgs.msg import Odometry


class OdometryToTUM:
    def __init__(self, topic: str, out_path: Path) -> None:
        self.topic = topic
        self.out_path = out_path
        self.num_poses_written = 0

        # create the output file
        self.out_path.parent.mkdir(parents=True, exist_ok=True)
        # Overwrite the file if it already exists
        if self.out_path.exists():
            rospy.logwarn(f"Overwrote existing file at '{self.out_path}'")
            self.out_path.unlink()
        self.out_path.touch()

        rospy.loginfo("Started odometry to TUM format logger.")

        # Initialize the ROS node
        rospy.init_node('odometry_to_tum', anonymous=True)

        # Subscribe to the /odom topic
        rospy.Subscriber(args.topic, Odometry, self.odometry_callback)

    def odometry_callback(self, msg: Odometry):
        # Extract the timestamp (in seconds and nanoseconds) from ROS time
        timestamp = msg.header.stamp.to_sec()

        # Extract position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        # Extract orientation as quaternion
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        # Format the data as TUM format: timestamp, x, y, z, qx, qy, qz, qw
        tum_entry = f"{timestamp} {x} {y} {z} {qx} {qy} {qz} {qw}\n"

        # Append the data to the file
        with open(self.out_path, "a") as file:
            file.write(tum_entry)

        self.num_poses_written += 1


def main(args):

    o = OdometryToTUM(args.topic, args.out_file)

    # Keep the node running
    while not rospy.is_shutdown():
        rospy.spin()

    rospy.loginfo(f"Wrote {o.num_poses_written} poses to '{args.out_file}'")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="ROS node to save odometry data in TUM format")
    parser.add_argument('--topic', type=str, help="ROS Odometry topic to subscribe to (e.g., /odom)")
    parser.add_argument('--out_file', type=Path, help="Path to the output file to save the TUM format data")
    args, _ = parser.parse_known_args()

    try:
        main(args)
    except rospy.ROSInterruptException:
        pass
