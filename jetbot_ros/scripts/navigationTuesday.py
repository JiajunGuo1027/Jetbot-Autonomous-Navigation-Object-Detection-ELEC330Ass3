#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import Int32MultiArray
import numpy as np
from Adafruit_MotorHAT import Adafruit_MotorHAT
from std_msgs.msg import String
import random
from std_msgs.msg import Bool
import sys


#functions and initalizations for motor control:
## ..................

motor_driver = Adafruit_MotorHAT(i2c_bus=1)

motor_left_ID = 1
motor_right_ID = 2

motor_left = motor_driver.getMotor(motor_left_ID)
motor_right = motor_driver.getMotor(motor_right_ID)

# sets motor speed between [-1.0, 1.0]
def set_speed(motor_ID, value):
	max_pwm = 115.0
	speed = int(min(max(abs(value * max_pwm), 0), max_pwm))

	if motor_ID == 2:
		motor = motor_left
	elif motor_ID == 1:
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
  
def backward(speed):
  set_speed(motor_left_ID,   speed)
  set_speed(motor_right_ID,  speed)

def forward(speed):
  set_speed(motor_left_ID,  -speed)
  set_speed(motor_right_ID, -speed)  
   
def stop():
  set_speed(motor_left_ID,  0)
  set_speed(motor_right_ID, 0)  
# initialization

# end of motor function intlaization 
#.................................................................





#TOF functions 

def getTOFData():
  #when this function is called it waits until data has been publihsed to the rostopic then reads it and returns the data
  TOFData =  rospy.wait_for_message('distance_topic', Int32MultiArray)
  TOFData = np.array( TOFData.data)
  
  TOFData = TOFData.reshape(8,8) # reshape to orgonal form
  
  return TOFData


#IMU functions

def accelerometer_callback(data):
	rospy.loginfo(rospy.get_caller_id() + "%s", data.data)

def magnetometer_callback(data):
	rospy.loginfo(rospy.get_caller_id() + "%s", data.data)

def listener():
	rospy.int_node('imu_data_listener', anonymous=Ture)

	rospy.Subscriber("accelerometer_data", String, accelerometer_callback)
	rospy.Subscriber("magnetometer_data", String, magnetometer_callback)

	rospy.spin()


def checkDuck(checks):
  #this function gets if the duck has been seen or not
  #if the duck has been seen it stops excution of the program 
  #returns true if the duck has not been seen
  # it checks a couple times if the duck has accualyl been seen
  # checks is the ammount of times it has checked 
  
  
  for i in range(checks):
    duckSeen =  rospy.wait_for_message('duck_detected', Bool)
    duckSeen = duckSeen.data
    if (not duckSeen):
      print(f"check {i} succesfull")
    else:
        print("checks unsuccesful")
    rospy.sleep(0.25)
  # this stops the execution of the program
  print("duck is found stop execution")
  sys.exit(1) 
  
    
  return False 
  




# navigation funcitons:


def moveAndCheckDuck(direction, speed, itterations, time):
  # this funiipon moves the robot and check if the duck was been found while moving
  # direction is a string, left, right, forward, backward
  # speed is betweek 0 and 1
  # itterations is the ammount of times the function will look for the duck
  # time is the time in seconds between itterations
  
  
  #f == forward
  #b == backward
  #l = left
  #r = right
  
   if (direction == "f"):
     for j in range(itterations):
       forward(speed)
       rospy.sleep(time)
       all_stop()
       checkDuck(3)
       
     
    
   # do if needed 
   


def navigate(currentAverage, thresholdArg, columbAveragesArg):
  # this function is so ugly rn fix later
  
  #average disance on the left or right side 
  rightDistance = np.mean(columbAveragesArg[0:])
  leftDistance = np.mean(columbAveragesArg[:7])
  
  # reverse and turn if disance is below threhold 
  # or if the very left / very right are too low
  if (currentAverage < thresholdArg or leftDistance < thresholdArg or rightDistance < thresholdArg):
    
    
    # move backward and check for duck at same time
    if (not lookForDuck()):
      # check if duck was found and stop if it was and return false 
      return False
    all_stop()
    for i in range(2):
      backward(0.75)
      rospy.sleep(0.25)
      all_stop()
      if (not lookForDuck()): 
        return False
      
    # turn left and chgeck for duck at same time
    for i in range(3):
      left(0.75)
      rospy.sleep(0.33)
      if (not lookForDuck()):
        return False
      
      all_stop()
      
      
    if (not lookForDuck()):
      # check if duck was found and stop if it was 
      return False
    
    # end of if statenebt]
    # if duck was not found reutnr True
    return True
  
  
  
  # move forward if average is above threshold 
  if (currentAverage > thresholdArg):
  
    for i in range(2):
      if (not lookForDuck()): 
        return False
      forward(0.75)
      rospy.sleep(0.25)
      if (not lookForDuck()): 
        return False
    return True
    
  
   


  

def  main():
    # inialize the node for subscibing to the TOF distance data
  rospy.init_node('distance_topic', anonymous=True)
  
  
  
 
  moveAndCheckDuck("f", 0.75, 3, 0.25)
  
  
    
    
    
    
    
  # loop to confirm duck has been found 
    
    
  print("duck was not found")
  all_stop()# stop the motors when scipt done


if __name__ == '__main__':
  main()
  os._exit(0)
  listener()
  
    
    
  
  
  

