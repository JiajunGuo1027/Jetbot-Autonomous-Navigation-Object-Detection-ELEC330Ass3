#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32

import smbus2
import RPi.GPIO as GPIO
import i2cEncoderMiniLib


def setup_encoders():
    GPIO.setmode(GPIO.BCM)
    bus = smbus2.SMBus(0)
    INT_pin = 4
    GPIO.setup(INT_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    
    encoderRight = i2cEncoderMiniLib.i2cEncoderMiniLib(bus, 0x20)
    encoderLeft = i2cEncoderMiniLib.i2cEncoderMiniLib(bus, 0x24)

    
    config = (i2cEncoderMiniLib.WRAP_ENABLE | i2cEncoderMiniLib.DIRE_RIGHT | i2cEncoderMiniLib.IPUP_ENABLE | i2cEncoderMiniLib.RMOD_X1)
    encoderRight.begin(config)
    encoderLeft.begin(config)
    return encoderLeft, encoderRight


def main():
    rospy.init_node('encoder_publisher', anonymous=True)
    pub_left = rospy.Publisher('encoder_left', Int32, queue_size=10)
    pub_right = rospy.Publisher('encoder_right', Int32, queue_size=10)
    rate = rospy.Rate(10)

    encoderLeft, encoderRight = setup_encoders()

    while not rospy.is_shutdown():
        left_value = encoderLeft.readCounter32()
        right_value = encoderRight.readCounter32()

        
        pub_left.publish(left_value)
        pub_right.publish(right_value)

        rospy.loginfo("Left Encoder: %d, Right Encoder: %d", left_value, right_value)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    finally:
        GPIO.cleanup()

