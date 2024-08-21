"""
This is a python class to control the camera stream.

Code adapted and modified from:
https://stackoverflow.com/questions/70597020/lower-latency-from-webcam-cv2-videocapture 
"""
__author__ = "Matt van Wijk"
__date__ = "20/08/2024"

import cv2
from threading import Thread, Condition


class CameraStream:
    def __init__(self, src='tcp://robo-retriever.local:8554', frame_h=720, frame_w=1280):
        """ Initialises the camera stream object """
        self.stopped = False
        self.stream = cv2.VideoCapture(src)
        self.stream.set(cv2.CAP_PROP_BUFFERSIZE, 0) # No buffer so we grab the most recent frame
        self.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_h)
        self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, frame_w)

        (self.grabbed, self.frame) = self.stream.read()
    
        self.hasNew = self.grabbed
        self.condition = Condition()

    def start(self):
        """ Starts the capture process """
        Thread(target=self.update, args=()).start()
        return self

    def update(self,):
        """ Continually reads from the stream for a new frame """
        while True:
            if self.stopped: return
            
            (self.grabbed, self.frame) = self.stream.read()
            with self.condition:
                self.hasNew = True
                self.condition.notify_all()    

    def read(self):
        """ Reads a frame from the stream """
        if not self.hasNew:
            with self.condition:
                self.condition.wait()

        self.hasNew = False
        return self.frame

    def stop(self):
        """ Stops the program from reading the stream """
        self.stopped = True
