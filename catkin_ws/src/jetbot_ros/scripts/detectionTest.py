import cv2
import numpy as np
import rospy
from std_msgs.msg import Bool

def gstreamer_pipeline(capture_width=320, capture_height=240, display_width=320, display_height=240, framerate=5, flip_method=0):
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

confThreshold = 0.2
modelConfiguration = "MobileNetSSD_deploy.prototxt"
modelWeights = "MobileNetSSD_deploy.caffemodel"
classesFile = "coco.names"

with open(classesFile, 'rt') as f:
    classes = f.read().rstrip('\n').split('\n')

net = cv2.dnn.readNetFromCaffe(modelConfiguration, modelWeights)
cap = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print("Failed to open camera.")
    rospy.signal_shutdown("Failed to open camera.")
    exit()

while not rospy.is_shutdown():
    ret, frame = cap.read()
    if not ret:
        print("Unable to read frame from the camera. Skipping...")
        continue

    blob = cv2.dnn.blobFromImage(frame, 0.007843, (300, 300), 127.5)
    net.setInput(blob)
    detections = net.forward()

    duck_detected = False
    for i in range(detections.shape[2]):
        confidence = detections[0, 0, i, 2]
        if confidence > confThreshold:
            idx = int(detections[0, 0, i, 1])
            if classes[idx] == 'bird':  # Assuming 'bird' as proxy for 'duck' for demonstration
                duck_detected = True
                break

    pub.publish(not duck_detected)  # Publish False if duck is detected, True otherwise

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

