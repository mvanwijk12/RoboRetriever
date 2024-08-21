# This is the main code
import cv2
import numpy as np
from ultralytics import YOLO
import time


# Loop code
while True:
    # In ball detection mode
    ## Capture image        
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
        class_names = model.names 

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

        ## ball detected?
        ball_centre = NULL
        if sports_ball_boxes != []:
            # get ball x and y coords
            # get ball centre
            # subtract ball horizontal coordinate from "centreline"
            # P control angle to target and drive forward
            # repeat capture image stuff above
            
            # Detect when reached target
            # if(targetReached):
            