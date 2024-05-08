#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import smbus2
import time
from lsm303d import LSM303D

def magnetometer_talker():
    pub = rospy.Publisher('magnetometer_data', String, queue_size=10)
    rospy.init_node('magnetometer_publisher', anonymous=True)
    rate = rospy.Rate(50)  # 50 Hz

    bus = smbus2.SMBus(0)
    lsm = LSM303D(0x1d, bus)  # Change to 0x1e if address jumper is soldered

    while not rospy.is_shutdown():
        xyz = lsm.magnetometer()
        magnetometer_data_str = "{:+06.2f} : {:+06.2f} : {:+06.2f}".format(*xyz)
        rospy.loginfo(magnetometer_data_str)
        pub.publish(magnetometer_data_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        magnetometer_talker()
    except rospy.ROSInterruptException:
        pass

