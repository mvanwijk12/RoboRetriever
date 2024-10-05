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
import torch
from ultralytics.engine.results import Boxes, Masks
import json


class Inference:
    BOX_CLS = 0
    LINES_CLS = 1 
    TENNIS_BALL_CLS = 2
    CLASS_NAMES = ['box', 'line', 'tennis-ball']

    def __init__(self, cs_stream, model_path='best.pt', frame_h=720, frame_w=1280):
        """ Initialises the inference object """
        self.stopped = False
        self.has_new = False
        # self.stream = cs.CameraStream(src=src, frame_h=frame_h, frame_w=frame_w).start()
        self.stream = cs_stream
        self.frame_h = frame_h
        self.frame_w = frame_w
        self.condition = Condition()
        self.frame = None
        self.model = YOLO(model_path)
        # self.img_frame = None
        self.num_counter_above_threshold = 0 # counts the number of detections within distance_threshold
        self.start_time_above_threshold = 0 # records the start time of the first detection within distance_threshold
        self.num_counter_critial_value = 3 # robot stops once num_counter_above_threshold == num_counter_critial_value
        self.distance_threshold = 30e-2 # in metres
        self.time_threshold = 120 # in seconds
        self.stop_condition = False
        self.logger = logging.getLogger(__name__)
        self.ld = LineDetector()

    def start(self):
        """ Starts the inference engine """
        Thread(target=self.process_image_update, args=()).start()
        return self

    def process_image_update(self, conf=0.4):
        """ Function to be run in Thread to continually process image stream and execute inference. Sets self.tennis_ball_box_detections
        and self.line_detections attributes.

        Parameters:
        - conf: inference confidence threshold. Inference results with confidence < conf, will be filtered out 
        
        Returns:
         - Does not return, remains in infinite loop
        """
        while True:
            if self.stopped: return

            # Capture frame
            self.frame = self.stream.read()

            # Run inference
            result = self.model.predict(source=self.frame, conf=conf, verbose=False)[0]

            # Apply the mask to filter the detections
            self.tennis_ball_box_detections = result.new()
            tennis_ball_box_mask = (result.boxes.data[:, -1] == self.BOX_CLS) | (result.boxes.data[:, -1] == self.TENNIS_BALL_CLS)
            boxes = result.boxes.data[tennis_ball_box_mask]
            self.tennis_ball_box_detections.boxes = Boxes(boxes, result.orig_shape)

            self.line_detections = result.new()
            if result.masks is not None:
                masks = result.masks.data[result.boxes.data[:, -1] == self.LINES_CLS]
            else:
                masks = torch.tensor([])
            self.line_detections.masks = Masks(masks, result.orig_shape)
            
            # Retrieve the image
            # self.img_frame = result.orig_img
            
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
        return self.tennis_ball_box_detections, self.line_detections
    
    def read_plot(self):
        """ Reads the inference results, evaluates the court line equations, plots the bboxes of boxes and tennis ball detection,
          superimposes the line equations and the line trigger box and returns a formatted detection object to be sent to robot.

        Returns:
        - msg: a list of dictionaries containing the inference results from the tennis balls and boxes and the triggered line
          direction in a dictionary (see self._wrap_detections)
        """
        if not self.has_new:
            with self.condition:
                self.condition.wait()

        # Copy the filtered detection for processing
        self.has_new = False
        tennis_ball_box_detects = deepcopy(self.tennis_ball_box_detections)
        line_detects = deepcopy(self.line_detections)

        ld_lines = None
        if line_detects.masks is not None and len(line_detects.masks) > 0: 
            ld_lines = self.ld.detect(line_detects.masks)
        
        self._plot_detections(tennis_ball_box_detects, ld_lines)
        return self._wrap_detections(tennis_ball_box_detects, ld_lines)
    
    def _calculate_error_from_centre(self, bbox_xyxy):
        """ Calculates the error of a bounding box from the vertical centreline of the image """
        return 1/2*(bbox_xyxy[0] + bbox_xyxy[2]) - self.frame_w/2

    def _plot_detections(self, tennis_ball_box_detects, ld_lines):
        """ Plots the bboxes of boxes and tennis ball detection, superimposes the line equations and the line trigger box.
        
        Parameters:
        - tennis_ball_box_detects: ultralytics.engine.results object containing inference results of the tennis balls and boxes
        - ld_lines: (triggered, largest_line_index, all_lines, directions) tuple returned from linedetection.LineDector.detect

        Returns:
        - None
        """
        annotated_image = tennis_ball_box_detects.plot()
        annotated_image = cv2.resize(annotated_image, (self.ld.frame_width, self.ld.frame_height))
        # top_left = (int(top_left[0] * 1280/640), int(top_left[1] *720/384))
        # bottom_right = (int(bottom_right[0] * 1280/640), int(bottom_right[1] *720/384))
        annotated_image = cv2.rectangle(annotated_image, self.ld.top_left, self.ld.bottom_right, (128, 0, 128), 2)
        
        if ld_lines is not None:
            colours = [(0, 255, 0), (0, 165, 255), (0, 0, 255)] # BGR format
            triggered, _, all_lines, _ = ld_lines
            for i in range(len(all_lines)):
                line = all_lines[i]
                annotated_image = cv2.line(annotated_image, (line[0], line[1]), (line[2], line[3]), colours[triggered[i]], 2)
                # annotated_image = cv2.line(annotated_image, (round(line[0] * 1280/640), round(line[1] * 720/384)), (round(line[2] * 1280/640), round(line[3] * 720/384)), colours[triggered[i]], 2)

        cv2.imshow('annotated_img', annotated_image)
        cv2.waitKey(5) 

    def _wrap_detections(self, tennis_ball_box_detects, ld_lines):
        """ Wraps the detection in a format to be sent to robot.
         
        Parameters:
        - tennis_ball_box_detects: ultralytics.engine.results object containing inference results of the tennis balls and boxes
        - ld_lines: (triggered, largest_line_index, all_lines, directions) tuple returned from linedetection.LineDector.detect

        Returns:
        - msg: a list of dictionaries containing the inference results from the tennis balls and boxes and the triggered line direction in a dictionary

        Example msg object:
        msg = [{'name': 'tennis ball', 'class': 2, 'confidence': 0.84805, 'box': {'x1': 873.23071, 'y1': 488.5304, 'x2': 904.45947, 'y2': 516.15857}},
          {'name': 'box', 'class': 0, 'confidence': 0.61668, 'box': {'x1': 921.52319, 'y1': 377.49768, 'x2': 1249.85254, 'y2': 470.74341}},
            {'line_direction': None}]

        msg = [{'name': 'tennis ball', 'class': 2, 'confidence': 0.90089, 'box': {'x1': 109.82916, 'y1': 535.94666, 'x2': 216.55533, 'y2': 632.59021}},
          {'name': 'tennis ball', 'class': 2, 'confidence': 0.66197, 'box': {'x1': 82.20343, 'y1': 403.78745, 'x2': 113.4147, 'y2': 418.24826}},
            {'line_direction': tensor([-0.5857,  1.0000])}]
        """
        msg = json.loads(tennis_ball_box_detects.tojson())
        
        if ld_lines is not None: # if there is a line detected in the frame
            _, largest_line_index, _, directions = ld_lines
            if largest_line_index is not None: # if the line detected is in the trigger box
                msg.append(dict(line_direction=directions[largest_line_index]))
            else:
                msg.append(dict(line_direction=None))
        else:
            msg.append(dict(line_direction=None))

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
    cap = CameraStream(src='2024-09-28_16-01-10-validation-converted.mp4').start()
    inf = Inference(cap).start()
    while True:
        inf.logger.debug(inf.read_plot())
        