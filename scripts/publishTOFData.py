import smbus2
import time
import numpy
import vl53l5cx_ctypes as vl53l5cx
from vl53l5cx_ctypes import STATUS_RANGE_VALID, STATUS_RANGE_VALID_LARGE_PULSE

import rospy
from std_msgs.msg import Int32MultiArray



def talker(arrayToPublish):

  rospy.init_node('distance_publisher', anonymous=True)
  
  distance_publisher = rospy.Publisher('distance_topic', Int32MultiArray, queue_size=10)
  
  distance_msg = Int32MultiArray()
  
  distance_msg.data = arrayToPublish
  
  distance_publisher.publish(distance_msg)






#bit that makes LRF finder
bus = smbus2.SMBus(0)

print("Uploading firmware, please wait...")
vl53 = vl53l5cx.VL53L5CX(0x29, bus)
print("Done!")
vl53.set_resolution(8 * 8)
vl53.enable_motion_indicator(8 * 8)
# vl53.set_integration_time_ms(50)

# Enable motion indication at 8x8 resolution
vl53.enable_motion_indicator(8 * 8)

# Default motion distance is quite far, set a sensible range
# eg: 40cm to 1.4m

vl53.start_ranging()





if __name__ == '__main__':
  while True:
    if vl53.data_ready():
        data = vl53.get_data()
        
        
        distance = numpy.flipud(numpy.array(data.distance_mm).reshape((8, 8)))
        
        
        
        distOneD = distance.flatten()
            
            
        
        talker(distOneD)
        
        
  
  

