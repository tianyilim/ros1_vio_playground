"""
Dummy node publishing IMU messages at 1000 Hz and 2000 Hz to debug the imu_downsampling_node.
"""

import rospy
from sensor_msgs.msg import Imu


class DummyImuPubNode:
    def __init__(self) -> None:
        self.accel_pub = rospy.Publisher("/imu/accel", Imu, queue_size=10)
        self.gyro_pub = rospy.Publisher("/imu/gyro", Imu, queue_size=10)

        self.accel_timer = rospy.Timer(
            rospy.Duration(1.0/1000), self.accel_timer_callback)
        self.gyro_timer = rospy.Timer(
            rospy.Duration(1.0/2000), self.gyro_timer_callback)

    def accel_timer_callback(self, event) -> None:
        msg = Imu()
        msg.header.stamp = rospy.Time.now()
        self.accel_pub.publish(msg)

    def gyro_timer_callback(self, event) -> None:
        msg = Imu()
        msg.header.stamp = rospy.Time.now()
        self.gyro_pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("dummy_imu_pub_node")
    node = DummyImuPubNode()
    rospy.spin()
