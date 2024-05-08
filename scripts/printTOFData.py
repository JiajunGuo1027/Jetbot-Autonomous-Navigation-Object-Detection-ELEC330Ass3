import rospy
from std_msgs.msg import Int32MultiArray
import numpy
import time




def getTOFData():
  TOFData =  rospy.wait_for_message('distance_topic', Int32MultiArray)
  TOFData = numpy.array( TOFData.data)
  
  TOFData = TOFData.reshape(8,8) # reshape to orgonal form
  
  return TOFData





def main():
  
  #inilaise distance topic
  rospy.init_node('distance_topic', anonymous=True)
  
  while True:
    TOFData = getTOFData()
    print(TOFData)
    print(TOFData[0, 0])
    #for i in TOFData:
      #print(i)
    #print(TOFData)
    #print(type(TOFData))


if __name__ == '__main__':
  main()