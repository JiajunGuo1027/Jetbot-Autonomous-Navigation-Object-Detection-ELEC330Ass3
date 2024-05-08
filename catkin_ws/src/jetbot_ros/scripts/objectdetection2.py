import cv2
import numpy as np

def gstreamer_pipeline(capture_width=640, capture_height=480, display_width=640, display_height=480, framerate=30, flip_method=0):
    return (
        "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int){}, height=(int){}, format=(string)NV12, framerate=(fraction){}/1 ! "
        "nvvidconv flip-method={} ! "
        "video/x-raw, width=(int){}, height=(int){}, format=(string)BGRx ! "
        "videoconvert ! video/x-raw, format=(string)BGR ! appsink".format(
            capture_width, capture_height, framerate, flip_method, display_width, display_height
        )
    )

# Initialize the parameters
confThreshold = 0.2  # Confidence threshold
nmsThreshold = 0.4   # Non-maximum suppression threshold
inpWidth = 300       # Width of network's input image
inpHeight = 300      # Height of network's input image
modelConfiguration = "MobileNetSSD_deploy.prototxt"
modelWeights = "MobileNetSSD_deploy.caffemodel"

# Load names of classes
classesFile = "coco.names"
classes = None
with open(classesFile, 'rt') as f:
    classes = f.read().rstrip('\n').split('\n')

# Load our serialized model from disk
print("[INFO] loading model...")
net = cv2.dnn.readNetFromCaffe(modelConfiguration, modelWeights)

# Initialize video capture with GStreamer pipeline
cap = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print("Failed to open camera.")
    exit()

# Process each frame from the video
while True:
    # Get frame from the video
    ret, frame = cap.read()
    if not ret:
        print("Unable to read frame from the camera. Skipping...")
        continue

    # Create a 4D blob from a frame
    blob = cv2.dnn.blobFromImage(frame, 0.007843, (inpWidth, inpHeight), 127.5)

    # Set the new input for the network
    net.setInput(blob)

    # Run the forward pass to get output from the output layers
    detections = net.forward()

    # Loop over the detections
    for i in range(detections.shape[2]):
        # Get the confidence (probability) of the current object detection
        confidence = detections[0, 0, i, 2]

        # Filter out weak detections by ensuring the confidence is greater than the threshold
        if confidence > confThreshold:
            # Get the index of the class label from the detection
            idx = int(detections[0, 0, i, 1])

            # Compute the (x, y)-coordinates of the bounding box for the object
            box = detections[0, 0, i, 3:7] * np.array([frame.shape[1], frame.shape[0], frame.shape[1], frame.shape[0]])
            (startX, startY, endX, endY) = box.astype("int")

            # Draw the prediction on the frame
            label = "{}: {:.2f}%".format(classes[idx], confidence * 100)
            cv2.rectangle(frame, (startX, startY), (endX, endY), (0, 255, 0), 2)
            y = startY - 15 if startY - 15 > 15 else startY + 15
            cv2.putText(frame, label, (startX, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Show the output frame
    cv2.imshow("Frame", frame)

    # Break the loop when the user clicks 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the capture when everything is done
cap.release()
cv2.destroyAllWindows()
import cv2
import numpy as np

def gstreamer_pipeline(capture_width=320, capture_height=240, display_width=320, display_height=240, framerate=30, flip_method=0):
    return (
        "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int){}, height=(int){}, format=(string)NV12, framerate=(fraction){}/1 ! "
        "nvvidconv flip-method={} ! "
        "video/x-raw, width=(int){}, height=(int){}, format=(string)BGRx ! "
        "videoconvert ! video/x-raw, format=(string)BGR ! appsink".format(
            capture_width, capture_height, framerate, flip_method, display_width, display_height
        )
    )

# Initialize the parameters
confThreshold = 0.2  # Confidence threshold
nmsThreshold = 0.4   # Non-maximum suppression threshold
inpWidth = 300       # Width of network's input image
inpHeight = 300      # Height of network's input image
modelConfiguration = "MobileNetSSD_deploy.prototxt"
modelWeights = "MobileNetSSD_deploy.caffemodel"

# Load names of classes
classesFile = "coco.names"
classes = None
with open(classesFile, 'rt') as f:
    classes = f.read().rstrip('\n').split('\n')

# Load our serialized model from disk
print("[INFO] loading model...")
net = cv2.dnn.readNetFromCaffe(modelConfiguration, modelWeights)

# Initialize video capture with GStreamer pipeline
cap = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print("Failed to open camera.")
    exit()

# Process each frame from the video
while True:
    # Get frame from the video
    ret, frame = cap.read()
    if not ret:
        print("Unable to read frame from the camera. Skipping...")
        continue

    # Create a 4D blob from a frame
    blob = cv2.dnn.blobFromImage(frame, 0.007843, (inpWidth, inpHeight), 127.5)

    # Set the new input for the network
    net.setInput(blob)

    # Run the forward pass to get output from the output layers
    detections = net.forward()

    # Loop over the detections
    for i in range(detections.shape[2]):
        # Get the confidence (probability) of the current object detection
        confidence = detections[0, 0, i, 2]

        # Filter out weak detections by ensuring the confidence is greater than the threshold
        if confidence > confThreshold:
            # Get the index of the class label from the detection
            idx = int(detections[0, 0, i, 1])

            # Compute the (x, y)-coordinates of the bounding box for the object
            box = detections[0, 0, i, 3:7] * np.array([frame.shape[1], frame.shape[0], frame.shape[1], frame.shape[0]])
            (startX, startY, endX, endY) = box.astype("int")

            # Draw the prediction on the frame
            label = "{}: {:.2f}%".format(classes[idx], confidence * 100)
            cv2.rectangle(frame, (startX, startY), (endX, endY), (0, 255, 0), 2)
            y = startY - 15 if startY - 15 > 15 else startY + 15
            cv2.putText(frame, label, (startX, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Show the output frame
    cv2.imshow("Frame", frame)

    # Break the loop when the user clicks 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the capture when everything is done
cap.release()
cv2.destroyAllWindows()
