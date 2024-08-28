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
import time
import logging


class Inference:
    def __init__(self, src='tcp://robo-retriever.local:8554', model_path='../../models/yolov8m.pt', frame_h=720, frame_w=1280):
        """ Initialises the camera stream object """
        self.stopped = False
        self.has_new = False
        self.stream = cs.CameraStream(src=src, frame_h=frame_h, frame_w=frame_w).start()
        self.condition = Condition()
        self.frame = None
        self.model = YOLO(model_path)
        self.desired_class_ids = [key for key, value in self.model.names.items() if value == 'sports ball']
        self.img_frame = None
        self.num_counter_above_threshold = 0 # counts the number of detections within distance_threshold
        self.start_time_above_threshold = 0 # records the start time of the first detection within distance_threshold
        self.num_counter_critial_value = 3 # robot stops once num_counter_above_threshold == num_counter_critial_value
        self.distance_threshold = 40e-2 # in metres
        self.time_threshold = 120 # in seconds
        self.logger = logging.getLogger(__name__)

    def start(self):
        Thread(target=self.process_image_update, args=()).start()
        return self

    def process_image_update(self, conf=0.2):
        while True:
            if self.stopped: return

            # Capture frame
            self.frame = self.stream.read()

            # Run inference
            result = self.model.track(source=self.frame, persist=True, conf=conf, verbose=False)[0]
            self.class_names = result.names

            # Convert to sv detection object
            detections = sv.Detections.from_ultralytics(result)
            if result.boxes.id is not None:
                detections.tracker_id = result.boxes.id.cpu().numpy().astype(int) # TODO: replace with cuda instead of cpu()?

            # Filter detections to only keep desired classes using mask
            mask = np.isin(detections.class_id, self.desired_class_ids)

            # Apply the mask to filter the detections
            self.filtered_detections = sv.Detections(
                xyxy=detections.xyxy[mask],
                confidence=detections.confidence[mask],
                class_id=detections.class_id[mask],
                tracker_id=detections.tracker_id[mask] if detections.tracker_id is not None else None,
                data={key: value[mask] for key, value in detections.data.items()} if detections.data else None
            )

            # Retrieve the image
            self.img_frame = result.orig_img
            
            # We have a new frame
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

        labels = []
        if len(self.filtered_detections.xyxy) > 0:
            # labels = [f"error: {round(1/2*(self.filtered_detections.xyxy[0][0] + self.filtered_detections.xyxy[0][2])-1280/2, 2)}"]
            labels = [
                    f"#{self.class_names[class_id]} {confidence:0.2f} error: {1/2*(xyxy[0] + xyxy[2])-1280/2:0.2f}"
                    for class_id, confidence, xyxy
                    in zip(self.filtered_detections.class_id, self.filtered_detections.confidence, self.filtered_detections.xyxy)
                ]

        annotated_image = box_annotator.annotate(scene=self.img_frame, detections=self.filtered_detections)
        annotated_image = label_annotator.annotate(scene=annotated_image, detections=self.filtered_detections, labels=labels)

        # Step 4: Plot the image with bounding boxes
        cv2.imshow('annotated_img', annotated_image)
        cv2.waitKey(5) 

        if len(self.filtered_detections.xyxy) == 1:
            x = 1/2*(self.filtered_detections.xyxy[0][0] + self.filtered_detections.xyxy[0][2]) - 1280/2
            dist = self.estimate_distance(self.filtered_detections.xyxy[0])
            self.logger.info(f'distance is {dist}')

            if dist < self.distance_threshold:
                self.logger.debug('Tennis ball detected within distance threshold')

                # Check detections are close by in time
                current_time = time.time()
                if (current_time - self.start_time_above_threshold < self.time_threshold):
                    # update num detections
                    self.num_counter_above_threshold += 1
                    self.logger.debug(f'#Detection Counter Incremented {self.num_counter_above_threshold}')
                else:
                    self.start_time_above_threshold = current_time
                    self.num_counter_above_threshold = 1
                    self.logger.debug('#Detection Counter Reset')

                # Check stop condition
                if self.num_counter_above_threshold >= self.num_counter_critial_value:
                    self.logger.info('Stop condition reached! Tennis ball detected!')
                    return dict(error=str(x), stop='True')
                
            return dict(error=str(x), stop='False')
        

    def estimate_distance(self, bbox, focal_pixel=770, real_world_diameter=67e-3):
        """
        Estimate the distance of the tennis ball from the camera.

        Parameters:
        - bbox: tuple or list (x1, y1, x2, y2) of the bounding box coordinates.
        - focal_length: float, the focal length of the camera in pixels.
        - real_world_diameter: float, the real-world diameter of the tennis ball in cm.

        Returns:
        - distance: float, estimated distance to the tennis ball in cm.
        """
        # Extract bounding box width and height
        x1, y1, x2, y2 = bbox
        bbox_width = abs(x2 - x1)

        # Calculate distance using the formula: Distance = (Real Diameter * Focal Length) / Perceived Diameter
        return (real_world_diameter * focal_pixel) / bbox_width


if __name__ == "__main__":
    logging.config.fileConfig('log.conf')
    logger = logging.getLogger(__name__)
    cap = Inference().start()
    while True:
        cap.logger.debug(cap.read_plot())