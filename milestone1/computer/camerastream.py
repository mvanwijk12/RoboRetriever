"""
This is a python class to control the camera stream.

Code adapted and modified from:
https://stackoverflow.com/questions/70597020/lower-latency-from-webcam-cv2-videocapture 
"""
__author__ = "Matt van Wijk"
__date__ = "20/08/2024"

import cv2
from threading import Thread, Condition, Lock, current_thread
from copy import deepcopy


class CameraStream:
    def __init__(self, src='tcp://robo-retriever.local:8554', frame_h=720, frame_w=1280):
        """ Initialises the camera stream object """
        self.stopped = False
        self.stream = cv2.VideoCapture(src)
        self.stream.set(cv2.CAP_PROP_BUFFERSIZE, 0) # No buffer so we grab the most recent frame
        self.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_h)
        self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, frame_w)
        (self.grabbed, self.frame) = self.stream.read()
        self.has_new = []
        self.has_new_index = {}
        self.condition = Condition()
        self.lock = Lock()

    def start(self):
        """ Starts the capture process """
        Thread(target=self.update, args=(), name='CameraStream').start()
        return self

    def update(self,):
        """ Continually reads from the stream for a new frame """
        while True:
            if self.stopped: return
            
            (self.grabbed, self.frame) = self.stream.read()
            with self.condition:
                self.update_frame_status()
                self.condition.notify_all()    

    def read(self):
        """ Reads a frame from the stream """
        thread_name = current_thread().name

        if not self.has_new_frame(thread_name):
            with self.condition:
                self.condition.wait()

        # Acquire a lock to safely access the frame
        with self.lock:
            frame = deepcopy(self.frame) # Create a copy 
            self.has_new[self.has_new_index[str(thread_name)]] = False  # Ensure has_new is reset by only one thread
        
        return frame

    def has_new_frame(self, thread_name):
        """ Checks if there is a new frame ready for the current thread """
        try:
            return self.has_new[self.has_new_index[str(thread_name)]]
        except KeyError:
            self.has_new_index[str(thread_name)] = len(self.has_new)
            self.has_new += [True]
            return True

    def update_frame_status(self):
        """ Updates the indicators for each thread """
        for i in range(len(self.has_new)):
            self.has_new[i] = True

    def stop(self):
        """ Stops the program from reading the stream """
        self.stopped = True


if __name__ == "__main__":
    cap = CameraStream().start()
    while(True):
        try:
            frame = cap.read()
            cv2.imshow('frame', frame)
            cv2.waitKey(1)
        except (Exception, KeyboardInterrupt) as e:
            cap.stop()
            raise e