#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import Int32MultiArray
import numpy as np
from Adafruit_MotorHAT import Adafruit_MotorHAT
from std_msgs.msg import String
import random
from std_msgs.msg import Bool

#functions and initalizations for motor control:


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


#things to code
#after it is done tunring left it double checks it is not still facing a wall
#the further it is away from somthing the quciker it goes 
#reverse when it sees an object so it does not hit the wall wiht its bunda
#depending on which side the object is decices which was it turns

def getTOFData():
  #when this function is called it waits until data has been publihsed to the rostopic then reads it and returns the data
  TOFData =  rospy.wait_for_message('distance_topic', Int32MultiArray)
  TOFData = np.array( TOFData.data)
  
  TOFData = TOFData.reshape(8,8) # reshape to orgonal form
  
  return TOFData

def getDuckTF():
  #when this function is called it waits until data has been publihsed to the rostopic then reads it and returns the data
  # false if there is a duck true if there is not 
  duckDetected =  rospy.wait_for_message('duck_detected', Bool)
  duckDetected = duckDetected.data
  return duckDetected
  
  
def lookForDuck():
  foundDuck = getDuckTF()
  if (not foundDuck):
      all_stop()
      print("found the duck!!!!")
  return foundDuck
      

def driveDynamic(thresholdArr, columbAveragesArg):
  #this function takes an array of thresholds
  #and the averages of all of the coliumbs
  #then will drive the robot based on the data it has
  #eg if the TOF sensor gives really hgih distancs it will drive far and if TOF gives low values will reverse then turn
  
  #isClose = np.any(columbAveragesArg < thresholdArr[0]) # if any values are below 150 will output true
  
  #get an array with all of the values which are below the threshold
  smallValues = np.empty((0,), dtype=float)
  print(columbAveragesArg)
  for j in columbAveragesArg:
    if (j <= thresholdArr[0]):
      smallValues = np.append(smallValues, j)
      
  
  
  numSmallValues = len(smallValues) # the number of values below the threshold
  leftAverage = np.mean(columbAveragesArg[:4]) # average of the first 4 columbs (left half of the lidar
  rightAverage = np.mean(columbAveragesArg[4:])
  totalAverage = np.mean(columbAveragesArg)
  if (totalAverage < thresholdArr[0]):
    #if it is below the smallest threshold 
    #reverse a little so it does not hit the wall wiht its back
    #go left
    #then exit function so loop restarts and can check if it is safe to move forward 
    backward(0.75)
    rospy.sleep(0.25)
    
    #now based on which side is closer either turn left or right
    if (leftAverage >= rightAverage):
      #if left is further away turn left
      left(0.75)
      rospy.sleep(0.75) # need this so it does not get stuck in corners 
    if(rightAverage > leftAverage):
      right(0.75)
      rospy.sleep(0.25)# need this so it does not get stuck in corners 
    
  #this bit is needed so it does not get snagged on walls
    if (leftAverage < 100):
      left(0.75)
      rospy.sleep(0.75) # need this so it does not get stuck in corners
      
    if (rightAverage < 100):
      right(0.75)
      rospy.sleep(0.25)# need this so it does not get stuck in corners
      
      
  #make it go forward if the lidar value is big engohg
  if (totalAverage > thresholdArr[0] ):
    forward(0.75)
  



def controlMotors(historicalAveragesArg, currentAverage, thresholdArg, columbAveragesArg):
  histAveMean = historicalAveragesArg.mean()
  
  #average disance on the left or right side 
  rightAverage = np.mean(columbAveragesArg[4:])
  leftAverage = np.mean(columbAveragesArg[:4])
  
  if (currentAverage < thresholdArg or rightAverage < thresholdArg or leftAverage < thresholdArg):
    
    
    if (not lookForDuck):
      return None
    all_stop()
    backward(1)
    rospy.sleep(0.5)
    #reverse for a bit so it does not hit its back on somthing 
    left(0.75)
    rospy.sleep(1)
    all_stop()
    return None
    
  
  if (currentAverage > thresholdArg):
    if(not lookForDuck()):
      return None
    forward(0.75)
    rospy.sleep(0.5)
    if(not lookForDuck()):
      return None
    all_stop()
    return None
  
   
  #if (histAveMean > thresholdArg):
   # forward(0.75)
    #rospy.sleep(1)
   # all_stop()
   # return None
    
   
  #if (histAveMean > thresholdArg*2):
    #forward(0.75)
   # rospy.sleep(3)
   # all_stop()
   # return None



  

  
  


if __name__ == '__main__':
  
  
  # inialize the node for subscibing to the TOF distance data
  rospy.init_node('distance_topic', anonymous=True)
  
  all_stop()# stop motors just incase
  
  foundDuck = True # will get from a rostopic when image processing is done 
  #allocate it from the topic in the loop
  
  thresholdMain = 120
  #threshold for the TOF in mm
  
  
 
  
  historicalAverages = np.array([0, 0, 0, 0, 0]) # store the last 5 averages of the distance
  #this array is first in last out
  counter = 0 #counter variable so can keep track of how to put the averages in the array
  hisAveLen = len(historicalAverages)
  
  for i in range(hisAveLen - 1):
    #fill the historical averages array before getting into the main loop
    distanceArr = getTOFData()
    historicalAverages[i] = distanceArr.mean()
  
  
  while True:
    foundDuck = lookForDuck()
    if (not lookForDuck):
      break;
    print("the value of duck TF is:")
    print(foundDuck)
    distanceArr = getTOFData()
    averageDistance = distanceArr.mean()
    
    columbAverages = np.median(distanceArr, axis=0) # columb 8 is the columb on the left
    # doing median not mean so outlier values do not affect the result
  
    # use colbum averages to decide which direction to turn
    historicalAverages[counter] = averageDistance
    counter = counter + 1
    if (counter >= hisAveLen - 1):
      counter = 0
      #reset counter to 0 so will replace the oldest value that was added to the array
    
    foundDuck = lookForDuck()
    if (foundDuck):
      #80mm was used as a starting point for the threshold
      if (not lookForDuck):
        break;
      controlMotors(historicalAverages, averageDistance, thresholdMain, columbAverages)
      
    
    
    foundDuck = lookForDuck()
    if (not foundDuck):
      all_stop()
      print("found the duck!!!!")
      break; # break the loop after duck if found
  
  all_stop()# stop the motors when scipt done
    
    
  
  
  

