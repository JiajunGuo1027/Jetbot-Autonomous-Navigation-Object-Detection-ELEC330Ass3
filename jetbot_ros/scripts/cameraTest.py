import rospy
from std_msgs.msg import Bool
import cv2
import numpy as np

# Set up GStreamer pipeline parameters for Raspberry Pi Camera V2.1
def gstreamer_pipeline(capture_width=640, capture_height=480, display_width=640, display_height=480, framerate=30, flip_method=0):
    return (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )

def detect_yellow_duck(frame):
    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # Define range of yellow color in HSV
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([30, 255, 255])
    # Threshold the HSV image to get only yellow colors
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    # Bitwise-AND mask and original image
    result = cv2.bitwise_and(frame, frame, mask=mask)
    # Check if there is more yellow than a threshold
    return np.sum(mask) > 50000  # adjust this threshold based on your needs

def main():
    rospy.init_node('camera_test', anonymous=True)
    pub = rospy.Publisher('duck_detected', Bool, queue_size=10)
    rate = rospy.Rate(10)  # 10hz
    cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)

    if not cap.isOpened():
        print("Failed to open camera.")
        rospy.signal_shutdown("Failed to open camera.")
        return

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture frame. Exiting...")
            break

        # Detect yellow duck
        duck_present = detect_yellow_duck(frame)

        # Publish the result (True if no duck, False if duck)
        pub.publish(not duck_present)

        cv2.imshow("CSI Camera", frame)
        keyCode = cv2.waitKey(30) & 0xFF
        if keyCode == 27:  # Stop the program on the ESC key
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

