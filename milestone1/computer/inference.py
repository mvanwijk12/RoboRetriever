"""
This is a python class to control image inference.

Ref. https://github.com/ultralytics/ultralytics/issues/10315
"""
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
from copy import deepcopy


class Inference:
    def __init__(self, cs_stream, model_path='../../models/yolov10m.pt', frame_h=720, frame_w=1280):
        """ Initialises the camera stream object """
        self.stopped = False
        self.has_new = False
        self.frame_h = frame_h
        self.frame_w = frame_w
        self.stream = cs_stream
        self.condition = Condition()
        self.frame = None
        self.model = YOLO(model_path)
        self.desired_class_ids = [key for key, value in self.model.names.items() if value == 'sports ball']
        self.img_frame = None
        self.num_counter_above_threshold = 0 # counts the number of detections within distance_threshold
        self.start_time_above_threshold = 0 # records the start time of the first detection within distance_threshold
        self.num_counter_critial_value = 3 # robot stops once num_counter_above_threshold == num_counter_critial_value
        self.distance_threshold = 30e-2 # in metres
        self.time_threshold = 120 # in seconds
        self.stop_condition = False
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

        # Copy the filtered detection for processing
        self.has_new = False
        filtered_detects = deepcopy(self.filtered_detections)
        image_frame = deepcopy(self.img_frame)

        # Create Annotators
        box_annotator = sv.BoxAnnotator() 
        label_annotator = sv.LabelAnnotator()

        n_detected_tennis_balls = len(filtered_detects.xyxy)
        self.logger.info(f'Number of detected tennis ball: {n_detected_tennis_balls}')
        
        # Handle the case there are no detections and tracker_id is None
        if filtered_detects.tracker_id is None:
            tracker_id_array = np.array([])
        else:
            tracker_id_array = filtered_detects.tracker_id

        labels = [
                f"#{tracker_id} {self.class_names[class_id]} {confidence:0.2f} error: {1/2*(xyxy[0] + xyxy[2]) - self.frame_w/2:0.0f}"
                for tracker_id, class_id, confidence, xyxy
                in zip(tracker_id_array, filtered_detects.class_id, filtered_detects.confidence, filtered_detects.xyxy)
            ]

        # Plot the image with bounding boxes
        if len(labels) == len(filtered_detects):
            annotated_image = box_annotator.annotate(scene=image_frame, detections=filtered_detects)
            annotated_image = label_annotator.annotate(scene=annotated_image, detections=filtered_detects, labels=labels)
            cv2.imshow('annotated_img', annotated_image)
            cv2.waitKey(5) 

        # If a tennis ball is detected, calculate the error and dist estimate
        if n_detected_tennis_balls > 0:

            # Calculate the errors
            x = 1/2*(filtered_detects.xyxy[:, 0] + filtered_detects.xyxy[:, 2]) - self.frame_w/2
            
            # Calculate the distances
            dist = self.estimate_distance(filtered_detects.xyxy)

            # Target the closest ball
            dist_min_idx = np.argmin(dist)
            
            self.logger.info(f'Distance is {dist[dist_min_idx]}')

            if dist[dist_min_idx] < self.distance_threshold:
                self.logger.debug('Tennis ball detected within distance threshold')

                # Check detections are close by in time
                current_time = time.time()
                if (current_time - self.start_time_above_threshold < self.time_threshold):
                    self.num_counter_above_threshold += 1
                    self.logger.debug(f'#Detection Counter Incremented {self.num_counter_above_threshold}')
                else:
                    self.start_time_above_threshold = current_time
                    self.num_counter_above_threshold = 1
                    self.logger.debug('#Detection Counter Reset')

                # Check stop condition
                if self.num_counter_above_threshold >= self.num_counter_critial_value:
                    self.stop_condition = True
                    
        
        if self.stop_condition:
            msg = dict(error='0', stop='True')
            self.logger.info(f'STOP CONDITION REACHED! TENNIS BALL DETECTED! msg = {msg}')
            return msg
        elif n_detected_tennis_balls > 0:
            msg = dict(error=str(x[dist_min_idx]), stop='False')
            self.logger.info(f'Tennis ball detected: msg = {msg}')
            return msg
        

    def estimate_distance(self, bboxs, focal_pixel=770, real_world_diameter=67e-3):
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
        bbox_widths = abs(bboxs[:, 2] - bboxs[:, 0])

        # Calculate distance using the formula: Distance = (Real Diameter * Focal Length) / Perceived Diameter
        return np.where(bbox_widths != 0, (real_world_diameter * focal_pixel) / bbox_widths, 1e-5)


if __name__ == "__main__":
    logging.config.fileConfig('log.conf')
    logger = logging.getLogger(__name__)
    src = 'tcp://robo-retriever.local:8554'
    frame_h = 720
    frame_w = 1280
    cs_stream = cs.CameraStream(src=src, frame_h=frame_h, frame_w=frame_w).start()
    cap = Inference(cs_stream=cs_stream, frame_h=frame_h, frame_w=frame_w).start()
    while True:
        cap.read_plot()