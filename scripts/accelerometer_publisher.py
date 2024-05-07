#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import smbus2
import time
from lsm303d import LSM303D

def talker():
    pub = rospy.Publisher('imu_data', String, queue_size=10)
    rospy.init_node('imu_publisher', anonymous=True)
    rate = rospy.Rate(50)  # 50hz

    bus = smbus2.SMBus(0)
    lsm = LSM303D(0x1d, bus)  # Change to 0x1e if address jumper is soldered

    while not rospy.is_shutdown():
        xyz = lsm.accelerometer()
        imu_data_str = "{:+06.2f}g : {:+06.2f}g : {:+06.2f}g".format(*xyz)
        rospy.loginfo(imu_data_str)
        pub.publish(imu_data_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

