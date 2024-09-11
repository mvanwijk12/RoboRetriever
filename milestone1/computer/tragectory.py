"""
This is a python class to detect and determine the new tragectory path of the robot when a line is
encoutered.
"""
__author__ = "Matt van Wijk"
__date__ = "05/09/2024"

from threading import Thread
import camerastream as cs
import numpy as np
import logging
import linedetection as ld
import time

class Tragectory:
    def __init__(self, cs_stream):
        self.logger = logging.getLogger(__name__)
        self.stream = cs_stream
        self.frame = None
        self.annotated_frame = None
        self.stopped = False

    def start(self):
        Thread(target=self.update_line_detection, args=()).start()
        return self

    def update_line_detection(self):
        " Detects any lines in the current frame "
        start_time = time.time
        while True:
            if self.stopped: return

            if (time.time - start_time > 1):
                
                start_time = time.time
                
                # Capture frame
                self.frame = self.stream.read()

                trigger, _, self.annotated_frame = ld.detect_line(self.frame)
                if trigger[0]: # if a line has entered trigger box
                    line_eq = trigger[3] # line equation vector [a, b]
                    self.logger.debug(f'Line equation: {line_eq}')
                    reflect_trag = self.reflected_tragectory(line_eq)
                    self.logger.info(f'Reflected tragectory: {reflect_trag}')

                self.logger.debug(f'No lines detected')

    def reflected_tragectory(line):
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
    src = 'tcp://robo-retriever.local:8554'
    frame_h = 720
    frame_w = 1280
    cs_stream = cs.CameraStream(src=src, frame_h=frame_h, frame_w=frame_w).start()
    trag = Tragectory(cs_stream=cs_stream).start()
    while True:
        try:
            pass
        except:
            trag.stop()
            raise Exception
    