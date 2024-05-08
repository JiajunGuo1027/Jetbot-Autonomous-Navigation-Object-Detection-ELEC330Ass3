
#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import smbus2
import time
from lsm303d import LSM303D

def talker():
    pub_accelerometer = rospy.Publisher('accelerometer_data', String, queue_size=10)
    pub_magnetometer = rospy.Publisher('magnetometer_data', String, queue_size=10)
    rospy.init_node('sensor_publisher', anonymous=True)
    rate = rospy.Rate(50)  # 50 Hz

    bus = smbus2.SMBus(0)
    lsm = LSM303D(0x1d, bus)  # Change to 0x1e if address jumper is soldered

    while not rospy.is_shutdown():
        accel_data = lsm.accelerometer()
        mag_data = lsm.magnetometer()

        accel_string = f"Accelerometer - X: {accel_data[0]:.2f}, Y: {accel_data[1]:.2f}, Z: {accel_data[2]:.2f}"
        mag_string = f"Magnetometer - X: {mag_data[0]:.2f}, Y: {mag_data[1]:.2f}, Z: {mag_data[2]:.2f}"

        rospy.loginfo(accel_string)
        rospy.loginfo(mag_string)

        pub_accelerometer.publish(accel_string)
        pub_magnetometer.publish(mag_string)

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
