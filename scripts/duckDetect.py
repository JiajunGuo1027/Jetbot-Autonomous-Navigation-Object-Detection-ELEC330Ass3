import cv2
import numpy as np
import rospy
from std_msgs.msg import Bool

def gstreamer_pipeline(capture_width=320, capture_height=240, display_width=320, display_height=240, framerate=30, flip_method=0):
    return (
        "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int){}, height=(int){}, format=(string)NV12, framerate=(fraction){}/1 ! "
        "nvvidconv flip-method={} ! "
        "video/x-raw, width=(int){}, height=(int){}, format=(string)BGRx ! "
        "videoconvert ! video/x-raw, format=(string)BGR ! appsink".format(
            capture_width, capture_height, framerate, flip_method, display_width, display_height
        )
    )

# Initialize ROS
rospy.init_node('duck_detector', anonymous=True)
pub = rospy.Publisher('duck_detected', Bool, queue_size=10)

cap = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print("Failed to open camera.")
    rospy.signal_shutdown("Failed to open camera.")
    exit()

# Define the yellow color range in HSV
#lower_yellow = np.array([240, 170, 40])
#upper_yellow = np.array([255, 195, 60])
#lower_yellow = np.array([190, 150, 0])
#upper_yellow = np.array([255, 220, 50])
#upper_yellow = np.array([255, 220, 10])
#lower_yellow = np.array([255, 210, 0])

lower_yellow = np.array([10, 150, 100])
upper_yellow = np.array([60, 255, 255])

#235, 159, 65
#255, 216, 1

while not rospy.is_shutdown():
    ret, frame = cap.read()
    if not ret:
        print("Unable to read frame from the camera. Skipping...")
        continue

    # Convert the frame to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # Create a mask for yellow color
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    # Bitwise-AND mask and original image
    yellow_output = cv2.bitwise_and(frame, frame, mask=mask)

    # Check if there is a significant yellow area
    if cv2.countNonZero(mask) > 1000:  # Threshold can be adjusted based on the size of the duck
        duck_detected = False  # Duck is detected
    else:
        duck_detected = True

    pub.publish(duck_detected)  # Publish False if duck is detected, True otherwise

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
