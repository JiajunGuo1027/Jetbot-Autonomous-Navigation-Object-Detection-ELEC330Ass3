#!/usr/bin/env python
import rospy
import time

from Adafruit_MotorHAT import Adafruit_MotorHAT
from std_msgs.msg import String




# sets motor speed between [-1.0, 1.0]
def set_speed(motor_ID, value):
	max_pwm = 115.0
	speed = int(min(max(abs(value * max_pwm), 0), max_pwm))

	if motor_ID == 1:
		motor = motor_left
	elif motor_ID == 2:
		motor = motor_right
	else:
		rospy.logerror('set_speed(%d, %f) -> invalid motor_ID=%d', motor_ID, value, motor_ID)
		return
	
	motor.setSpeed(speed)

	if value > 0:
		motor.run(Adafruit_MotorHAT.FORWARD)
	else:
		motor.run(Adafruit_MotorHAT.BACKWARD)





# stops all motors
def all_stop():
	motor_left.setSpeed(0)
	motor_right.setSpeed(0)

	motor_left.run(Adafruit_MotorHAT.RELEASE)
	motor_right.run(Adafruit_MotorHAT.RELEASE)





# simple string commands (left/right/forward/backward/stop)
def on_cmd_str(msg):
	rospy.loginfo(rospy.get_caller_id() + ' cmd_str=%s', msg.data)

	if msg.data.lower() == "left":
		set_speed(motor_left_ID,  -1.0)
		set_speed(motor_right_ID,  1.0) 
	elif msg.data.lower() == "right":
		set_speed(motor_left_ID,   1.0)
		set_speed(motor_right_ID, -1.0) 
	elif msg.data.lower() == "forward":
		set_speed(motor_left_ID,   1.0)
		set_speed(motor_right_ID,  1.0)
	elif msg.data.lower() == "backward":
		set_speed(motor_left_ID,  -1.0)
		set_speed(motor_right_ID, -1.0)  
	elif msg.data.lower() == "stop":
		all_stop()
	else:
		rospy.logerror(rospy.get_caller_id() + ' invalid cmd_str=%s', msg.data)


def left(speed):
  set_speed(motor_left_ID,  -speed)
  set_speed(motor_right_ID,  speed)

def right(speed):
  set_speed(motor_left_ID,   speed)
  set_speed(motor_right_ID, -speed)
  
def forward(speed):
  set_speed(motor_left_ID,   speed)
  set_speed(motor_right_ID,  speed)

def backward(speed):
  set_speed(motor_left_ID,  -speed)
  set_speed(motor_right_ID, -speed)  
   
def stop():
  set_speed(motor_left_ID,  0)
  set_speed(motor_right_ID, 0)  
# initialization



if __name__ == '__main__':

  # setup motor controller
  motor_driver = Adafruit_MotorHAT(i2c_bus=1)
  
  motor_left_ID = 2
  motor_right_ID = 1
  
  motor_left = motor_driver.getMotor(motor_left_ID)
  motor_right = motor_driver.getMotor(motor_right_ID)
  
  
  all_stop()
