"""
Simple ROS node that subscribes to two separate IMU topics, one for Accel and one for Gyro, fuses them, and republishes
at a lower rate.

Usage:
    python3 run imu_downsampling_node.py --rate 200.0 --imu_pub_topic /imu --accel_sub_topic /imu/accel --gyro_sub_topic /imu/gyro
"""

import argparse
import rospy
from sensor_msgs.msg import Imu
import message_filters
from typing import Optional
from threading import Lock


class ImuDownsamplingNode:
    def __init__(self, rate: float, imu_pub_topic: str, accel_sub_topic: str, gyro_sub_topic: str) -> None:

        rospy.loginfo(
            f"Subscribing to Accel msgs on [{accel_sub_topic}] and Gyro msgs on [{gyro_sub_topic}]")
        rospy.loginfo(
            f"Publishing combined IMU msgs on [{imu_pub_topic}] at rate {rate:0.2f} Hz")

        self.imu_pub = rospy.Publisher(imu_pub_topic, Imu, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(1.0/rate), self.timer_callback)

        self.message_filter_subs = [message_filters.Subscriber(accel_sub_topic, Imu, queue_size=1000, tcp_nodelay=True),
                                    message_filters.Subscriber(gyro_sub_topic, Imu, queue_size=1000, tcp_nodelay=True)]
        self.time_sync = message_filters.ApproximateTimeSynchronizer(
            self.message_filter_subs,
            100,  # queue size
            0.2  # slop
        )
        self.time_sync.registerCallback(self.accel_gyro_callback)

        self.imu_msg_lock = Lock()
        self.last_imu_msg: Optional[Imu] = None

    def timer_callback(self, event) -> None:
        if self.last_imu_msg is None:
            return

        # Read the last imu message
        with self.imu_msg_lock:
            last_imu_msg = self.last_imu_msg

        self.imu_pub.publish(last_imu_msg)

    def accel_gyro_callback(self, accel_msg: Imu, gyro_msg: Imu) -> None:
        imu_msg = Imu()
        # Take the latest timestamp
        imu_msg.header.stamp = accel_msg.header.stamp if accel_msg.header.stamp > gyro_msg.header.stamp else gyro_msg.header.stamp
        imu_msg.header.frame_id = 'imu'

        imu_msg.angular_velocity = gyro_msg.angular_velocity
        imu_msg.angular_velocity_covariance = gyro_msg.angular_velocity_covariance

        imu_msg.linear_acceleration = accel_msg.linear_acceleration
        imu_msg.linear_acceleration_covariance = accel_msg.linear_acceleration_covariance

        with self.imu_msg_lock:
            self.last_imu_msg = imu_msg


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--rate", type=float, default=200.0,
                        help="Rate at which to publish the IMU messages")
    parser.add_argument("--imu_pub_topic", type=str, default="/imu",
                        help="Topic to publish combined IMU messages")
    parser.add_argument("--accel_sub_topic", type=str, default="/imu/accel",
                        help="Topic to subscribe to Accel messages")
    parser.add_argument("--gyro_sub_topic", type=str, default="/imu/gyro",
                        help="Topic to subscribe to Gyro messages")
    args = parser.parse_args()

    rospy.init_node('imu_downsampling_node')
    node = ImuDownsamplingNode(
        args.rate, args.imu_pub_topic, args.accel_sub_topic, args.gyro_sub_topic)
    rospy.spin()