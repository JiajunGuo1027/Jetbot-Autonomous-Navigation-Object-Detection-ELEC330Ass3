#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import smbus2
import time
from lsm303d import LSM303D

def accelerometer_talker():
    pub = rospy.Publisher('accelerometer_data', String, queue_size=10)
    rospy.init_node('accelerometer_publisher', anonymous=True)
    rate = rospy.Rate(50)  # 50hz

    bus = smbus2.SMBus(0)
    lsm = LSM303D(0x1d, bus)  # Change to 0x1e if address jumper is soldered

    while not rospy.is_shutdown():
        xyz = lsm.accelerometer()
        accelerometer_data_str = "{:+06.2f}g : {:+06.2f}g : {:+06.2f}g".format(*xyz)
        rospy.loginfo(accelerometer_data_str)
        pub.publish(accelerometer_data_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        accelerometer_talker()
    except rospy.ROSInterruptException:
        pass

