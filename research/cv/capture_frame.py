import cv2
import numpy as np
from ultralytics import YOLO
import time

model = YOLO("yolov8n.pt")  # load an official detection model

# Define the TCP URL for the video stream
stream_url = 'tcp://robo-retriever.local:8554'

# Open the video stream using OpenCV
cap = cv2.VideoCapture(stream_url)

if not cap.isOpened():
    print("Error: Could not open video stream")
    exit()

while True:
    # Capture frame-by-frame
    for i in range(100):
        ret, frame = cap.read()

    if not ret:
        print("Failed to grab frame")
        break

    # Display the frame
    results = model.track(source=frame)

    # for result in results:
    #     result.show()
    #cv2.imshow('TCP Frame', frame)

    # Get the names of the classes from the model
    class_names = model.names  # Example: ['person', 'bicycle', 'car', ..., 'sports ball', ...]

    # Identify the index of 'sports ball' in class_names
    #print(class_names)
    sports_ball_index = 32

    # Filter out bounding boxes and other info for 'sports ball'
    sports_ball_boxes = []
    for result in results:  # results.xyxy[0] has [x1, y1, x2, y2, confidence, class]
        print(result.boxes.cls)
        print(result.boxes.conf)
        if result.boxes.cls[0] == sports_ball_index:
            sports_ball_boxes.append(result.boxes.xyxy)  # Append the bounding box [x1, y1, x2, y2]

    # sports_ball_boxes now contains bounding boxes of all detected sports balls in the image
    print(sports_ball_boxes)


    # Break the loop on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything is done, release the capture
cap.release()
cv2.destroyAllWindows()
