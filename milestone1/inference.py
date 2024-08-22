"""
This is a python class to control image inference.
"""
# Ref. https://github.com/ultralytics/ultralytics/issues/10315
__author__ = "Matt van Wijk"
__date__ = "20/08/2024"

from ultralytics import YOLO
from threading import Thread, Condition
import supervision as sv
import camerastream as cs
import numpy as np
import cv2
import matplotlib.pyplot as plt
from time import sleep

class Inference:
    def __init__(self, src='tcp://robo-retriever.local:8554', frame_h=720, frame_w=1280):
        """ Initialises the camera stream object """
        self.stopped = False
        self.position_error = 0
        self.has_new = False
        self.stream = cs.CameraStream(src=src, frame_h=frame_h, frame_w=frame_w).start()
        self.condition = Condition()
        self.frame = None
        self.bboxes = None
        self.model = YOLO("yolov8n.pt")
        self.desired_class_ids = [32]
        self.img_frame = None

    def start(self):
        Thread(target=self.process_image_update, args=()).start()
        return self

    def process_image_update(self, conf=0.5):
        while True:
            if self.stopped: return

            self.frame = self.stream.read()
            result = self.model.track(source=self.frame, persist=True, conf=conf)[0]
            # class_ids = result.boxes.cls.cpu().numpy().astype(int)
            # img_frame = result.orig_img

            # Convert to sv detection object
            detections = sv.Detections.from_ultralytics(result)
            if result.boxes.id is not None:
                detections.tracker_id = result.boxes.id.cpu().numpy().astype(int) # TODO: replace with cuda instead of cpu()?

            # Filter detections to only keep desired classes
            # self.filtered_detections = [d for d in detections if d.class_id in self.desired_class_ids]
            # self.img_frame = result.orig_img
            mask = np.isin(detections.class_id, self.desired_class_ids)

            # Step 2: Apply the mask to filter the detections
            self.filtered_detections = sv.Detections(
                xyxy=detections.xyxy[mask],
                confidence=detections.confidence[mask],
                class_id=detections.class_id[mask],
                tracker_id=detections.tracker_id[mask] if detections.tracker_id is not None else None,
                data={key: value[mask] for key, value in detections.data.items()} if detections.data else None
            )

            self.img_frame = result.orig_img
            
            # if a tennis ball is detected we have a new frame
            if len(self.filtered_detections.xyxy) > 0:
                with self.condition:
                    self.has_new = True
                    self.condition.notify_all()

    def read(self):
        """ Reads a frame from the stream """
        if not self.has_new:
            with self.condition:
                self.condition.wait()

        self.has_new = False 
        return self.filtered_detections
    
    def read_plot(self):
        if not self.has_new:
            with self.condition:
                self.condition.wait()

        self.has_new = False
        box_annotator = sv.BoxAnnotator()  # Create a BoxAnnotator
        label_annotator = sv.LabelAnnotator()

        #labels = []
        #if self.filtered_detections.tracker_id is not None:
        labels = [f"error: {round(1/2*(self.filtered_detections.xyxy[0][0] + self.filtered_detections.xyxy[0][2])-1280/2, 2)}"]

        annotated_image = box_annotator.annotate(scene=self.img_frame, detections=self.filtered_detections)
        annotated_image = label_annotator.annotate(scene=annotated_image, detections=self.filtered_detections, labels=labels)

        # Step 4: Plot the image with bounding boxes
        cv2.imshow('annotated_img', annotated_image)
        cv2.waitKey(1)  
        return None


if __name__ == "__main__":
    cap = Inference(src='tcp://raspberry.local:8554').start()
    while True:
        cap.read_plot()
        # cv2.imshow('frame', frame)
        # cv2.waitKey(1)