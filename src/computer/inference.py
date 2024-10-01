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
from camerastream import CameraStream
from linedetection import LineDetector


class Inference:
    BOX_CLS = 0
    LINES_CLS = 1 
    TENNIS_BALL_CLS = 2
    CLASS_NAMES = ['box', 'line', 'tennis-ball']

    def __init__(self, cs_stream, model_path='best.pt', frame_h=720, frame_w=1280):
        """ Initialises the camera stream object """
        self.stopped = False
        self.has_new = False
        # self.stream = cs.CameraStream(src=src, frame_h=frame_h, frame_w=frame_w).start()
        self.stream = cs_stream
        self.frame_h = frame_h
        self.frame_w = frame_w
        self.condition = Condition()
        self.frame = None
        self.model = YOLO(model_path)
        self.img_frame = None
        self.num_counter_above_threshold = 0 # counts the number of detections within distance_threshold
        self.start_time_above_threshold = 0 # records the start time of the first detection within distance_threshold
        self.num_counter_critial_value = 3 # robot stops once num_counter_above_threshold == num_counter_critial_value
        self.distance_threshold = 30e-2 # in metres
        self.time_threshold = 120 # in seconds
        self.stop_condition = False
        self.logger = logging.getLogger(__name__)
        self.ld = LineDetector()

    def start(self):
        Thread(target=self.process_image_update, args=()).start()
        return self

    def process_image_update(self, conf=0.5):
        while True:
            if self.stopped: return

            # Capture frame
            self.frame = self.stream.read()

            # Run inference
            result = self.model.track(source=self.frame, persist=True, conf=conf, verbose=False)[0]

            # Convert to sv detection object
            self.detections = sv.Detections.from_ultralytics(result)
            if result.boxes.id is not None:
                self.detections.tracker_id = result.boxes.id.cpu().numpy().astype(int) # TODO: replace with cuda instead of cpu()?

            # Filter detections to only keep desired classes using mask
            mask_box = np.isin(self.detections.class_id, self.BOX_CLS)
            mask_line = np.isin(self.detections.class_id, self.LINES_CLS)
            mask_tennis_ball = np.isin(self.detections.class_id, self.TENNIS_BALL_CLS)

            # Apply the mask to filter the detections
            self.box_detections = sv.Detections(
                xyxy=self.detections.xyxy[mask_box],
                confidence=self.detections.confidence[mask_box],
                class_id=self.detections.class_id[mask_box],
                tracker_id=self.detections.tracker_id[mask_box] if self.detections.tracker_id is not None else None,
                data={key: value[mask_box] for key, value in self.detections.data.items()} if self.detections.data else None
            )

            self.line_detections = sv.Detections(
                xyxy=self.detections.xyxy[mask_line],
                confidence=self.detections.confidence[mask_line],
                class_id=self.detections.class_id[mask_line],
                tracker_id=self.detections.tracker_id[mask_line] if self.detections.tracker_id is not None else None,
                data={key: value[mask_line] for key, value in self.detections.data.items()} if self.detections.data else None,
                mask=self.detections.mask[mask_line] if self.detections.mask is not None else None
            )

            self.tennis_ball_detections = sv.Detections(
                xyxy=self.detections.xyxy[mask_tennis_ball],
                confidence=self.detections.confidence[mask_tennis_ball],
                class_id=self.detections.class_id[mask_tennis_ball],
                tracker_id=self.detections.tracker_id[mask_tennis_ball] if self.detections.tracker_id is not None else None,
                data={key: value[mask_tennis_ball] for key, value in self.detections.data.items()} if self.detections.data else None
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
        return self.box_detections, self.line_detections, self.tennis_ball_detections
    
    def read_plot(self):
        if not self.has_new:
            with self.condition:
                self.condition.wait()

        # Copy the filtered detection for processing
        self.has_new = False
        box_detects = deepcopy(self.box_detections)
        line_detects = deepcopy(self.line_detections)
        tennis_ball_detects = deepcopy(self.tennis_ball_detections)
        detects = deepcopy(self.detections)
        image_frame = deepcopy(self.img_frame)

        # Create Annotators
        box_annotator = sv.BoxAnnotator() 
        label_annotator = sv.LabelAnnotator()

        n_detected_tennis_balls = len(tennis_ball_detects.xyxy)
        self.logger.info(f'Number of detected tennis ball: {n_detected_tennis_balls}')
        
        # Handle the case there are no detections and tracker_id is None
        if tennis_ball_detects.tracker_id is None:
            tennis_ball_tracker_id_array = np.array([])
        else:
            tennis_ball_tracker_id_array = tennis_ball_detects.tracker_id

        tennis_ball_labels = [
                f"#{tracker_id} {self.CLASS_NAMES[class_id]} {confidence:0.2f} error: {1/2*(xyxy[0] + xyxy[2]) - self.frame_w/2:0.0f}"
                for tracker_id, class_id, confidence, xyxy
                in zip(tennis_ball_tracker_id_array, tennis_ball_detects.class_id, tennis_ball_detects.confidence, tennis_ball_detects.xyxy)
            ]
        
        if box_detects.tracker_id is None:
            box_tracker_id_array = np.array([])
        else:
            box_tracker_id_array = tennis_ball_detects.tracker_id

        box_labels = [
                f"#{tracker_id} {self.CLASS_NAMES[class_id]} {confidence:0.2f} error: {1/2*(xyxy[0] + xyxy[2]) - self.frame_w/2:0.0f}"
                for tracker_id, class_id, confidence, xyxy
                in zip(box_tracker_id_array, box_detects.class_id, box_detects.confidence, box_detects.xyxy)
            ]

        # if detects.tracker_id is None:
        #     tracker_id_array = np.array([])
        # else:
        #     tracker_id_array = detects.tracker_id
        
        # labels = []
        # for i in range(len(tracker_id_array)):
        #     if detects.class_id[i] == self.TENNIS_BALL_CLS:
        #         labels.append(f"#{tracker_id_array[i]} {self.CLASS_NAMES[self.TENNIS_BALL_CLS]} {detects.confidence[i]:0.2f} error: {1/2*(detects.xyxy[i][0] + detects.xyxy[i][2]) - self.frame_w/2:0.0f}")
        #     else:
        #         labels.append(f'#{tracker_id_array[i]} {self.CLASS_NAMES[detects.class_id[i]]} {detects.confidence[i]:0.2f}')
        all_lines = []
        if line_detects.mask is not None and len(line_detects.mask) > 0:
            print(line_detects.mask)
            triggered, largest_line, all_lines = self.ld.detect(line_detects.mask)

        # Plot the image with bounding boxes
        if len(tennis_ball_labels) == len(tennis_ball_detects) and len(box_labels) == len(box_detects):
            annotated_image = box_annotator.annotate(scene=image_frame, detections=tennis_ball_detects)
            annotated_image = label_annotator.annotate(scene=annotated_image, detections=tennis_ball_detects, labels=tennis_ball_labels)
            annotated_image = box_annotator.annotate(scene=annotated_image, detections=box_detects)
            annotated_image = label_annotator.annotate(scene=annotated_image, detections=box_detects, labels=box_labels)
            for line in all_lines:
                annotated_image = cv2.line(annotated_image, (line[0], line[1]), (line[2], line[3]), (0, 255, 0), 2)
            cv2.imshow('annotated_img', annotated_image)
            cv2.waitKey(5) 

        # If a tennis ball is detected, calculate the error and dist estimate
        if n_detected_tennis_balls > 0:

            # Calculate the errors
            x = 1/2*(tennis_ball_detects.xyxy[:, 0] + tennis_ball_detects.xyxy[:, 2]) - self.frame_w/2
            
            # Calculate the distances
            dist = self.estimate_distance(tennis_ball_detects.xyxy)

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
    cap = CameraStream(src='2024-09-07_00-53-32-validation-converted.mp4').start()
    inf = Inference(cap).start()
    while True:
        inf.logger.debug(inf.read_plot())