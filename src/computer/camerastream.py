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
import logging
import logging.config
import time

class CameraStream:
    def __init__(self, src='tcp://robo-retriever.local:8554', frame_h=720, frame_w=1280):
        """ Initialises the camera stream object, set src=0 for testing with webcam """
        self.stopped = False
        self.has_new = []
        self.has_new_index = {}
        self.condition = Condition()
        self.lock = Lock()
        self.logger = logging.getLogger(__name__)

        # Setup cv2 VideoCapture
        self.stream = cv2.VideoCapture(src)
        self.stream.set(cv2.CAP_PROP_BUFFERSIZE, 0) # No buffer so we grab the most recent frame
        # self.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_h)
        # self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, frame_w)
        (self.grabbed, self.frame) = self.stream.read()
        
    def start(self):
        """ Starts the capture process """
        self.logger.info('Starting camera stream...')
        self.thread = Thread(target=self.update, args=(), name='CameraStream')
        self.thread.start()
        return self

    def update(self,):
        """ Continually reads from the stream for a new frame """
        while True:
            if self.stopped:
                self.logger.info(f'Stopping camera stream and releasing camera...')
                self.stream.release()
                return 
            
            (self.grabbed, self.frame) = self.stream.read()
            with self.condition:
                self.update_frame_status()
                self.condition.notify_all()    

    def read(self):
        """ Reads a frame from the stream """
        thread_name = current_thread().name
        self.logger.debug(f'Reading a frame for thread {thread_name}')

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
        if self.thread:
            self.thread.join()  # Wait for the thread to finish


if __name__ == "__main__":
    logging.config.fileConfig('log.conf')
    logger = logging.getLogger(__name__)

    cap = CameraStream(src=0).start()

    def test_function(cs_stream, name, stop):
        start_time = time.time()
        while time.time() - start_time < 5 and not stop():
            frame = cs_stream.read()
            cv2.imshow(name, frame)
            cv2.waitKey(1)
        cs_stream.stop()

    stop_flag = False
    thread1 = Thread(target=test_function, args=(cap, 'THREAD 1', lambda : stop_flag), name='thread1')
    thread2 = Thread(target=test_function, args=(cap, 'THREAD 2', lambda : stop_flag), name='thread2')

    try:
        print('starting thread1')
        thread1.start()
        print('starting thread2')
        thread2.start()

        # Keep the main program running while threads work
        while thread1.is_alive() or thread2.is_alive():
            time.sleep(0.1)

    except KeyboardInterrupt:
        # Handle Ctrl+C and set the stop flag
        print("Ctrl+C pressed, stopping threads...")
        stop_flag = True

        # Wait for threads to finish
        thread1.join()
        thread2.join()

        # Close any OpenCV windows
        cv2.destroyAllWindows()

    finally:
        cap.stop()