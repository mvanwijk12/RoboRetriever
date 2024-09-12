"""
This is a python class to detect and determine the new tragectory path of the robot when a line is
encoutered.
"""
__author__ = "Matt van Wijk"
__date__ = "05/09/2024"

from threading import Thread, Condition
import camerastream as cs
import numpy as np
import logging
import linedetection as ld
import time
import cv2
from copy import deepcopy

class Tragectory:
    def __init__(self, cs_stream):
        self.logger = logging.getLogger(__name__)
        self.stream = cs_stream
        self.frame = None
        self.annotated_frame = None
        self.stopped = False
        self.line_status = False

    def start(self):
        Thread(target=self.update_line_detection, args=()).start()
        return self

    def update_line_detection(self):
        " Detects any lines in the current frame "
        while True:
            if self.stopped: return
    
            # Capture frame
            self.frame = self.stream.read()
            
            trigger, _, frame = ld.detect_line(self.frame)
            
            if trigger[0]: # if a line has entered trigger box
                self.line_eq = trigger[3] # line equation vector [a, b]
                # self.logger.debug(f'Line equation: {self.line_eq}')
                self.reflect_trag = self.reflected_tragectory(self.line_eq)
                # self.logger.info(f'Reflected tragectory: {reflect_trag}')
                self.annotated_frame = deepcopy(frame)
                # We have a new line
                self.line_status = True

            # self.logger.debug(f'No lines detected')

    def read_reflect_trag(self):
        """ Reads a frame from the stream """
        if self.line_status:
            self.logger.info(f'LINE DETECTED {self.line_eq}, REFLECT TRAJ {self.reflect_trag}')
            self.line_status = False
            cv2.imshow('label', self.annotated_frame)
            cv2.waitKey(100)
        # return self.filtered_detections

    def reflected_tragectory(self, line):
        " Calculates the reflected tragectory of the robot when a line is detected "
        # Input is line in the form ax + by = c where line = [a, b]
        initial_direction = np.array([0,1]) # normalised direction
        normal_direction = line/np.linalg.norm(line) # normalised direction
        reflected_direction = initial_direction - (2*np.dot(initial_direction, normal_direction))*normal_direction
        
        return reflected_direction/np.linalg.norm(reflected_direction) # normalised direction
    
    def stop(self):
        self.stop = True

if __name__ == "__main__":
    logging.config.fileConfig('log.conf')
    logger = logging.getLogger(__name__)
    src = 0
    frame_h = 720
    frame_w = 1280
    cs_stream = cs.CameraStream(src=src, frame_h=frame_h, frame_w=frame_w).start()
    trag = Tragectory(cs_stream=cs_stream).start()
    
    while True:
        trag.read_reflect_trag()

    # try:
    #     # Keep the main program running while threads work
    #     while thread1.is_alive():
    #         time.sleep(0.1)
    # except KeyboardInterrupt:
    #     # Handle Ctrl+C and set the stop flag
    #     print("Ctrl+C pressed, stopping threads...")
    #     trag.stop()