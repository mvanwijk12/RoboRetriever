"""
This is a python class for creating a client

Modified from: https://realpython.com/python-sockets/ 
"""
__author__ = "Matt van Wijk"
__date__ = "28/08/2024"

import socket
import libclient
import time
from inference import Inference
import camerastream as cs
from tragectory import Tragectory
import logging
import logging.config
import sys

class ConnectionClient:
    MAX_RECONNECTION_ATTEMPTS = 10
    def __init__(self, hostname='robo-retriever.local', port=65432):
        self.host = hostname
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setblocking(False)
        self.logger = logging.getLogger(__name__)
        self.logger.info(f"Starting connection to {(self.host, self.port)}")
        self.sock.connect_ex((self.host, self.port))

    def close(self):
        ''' Closes the connection '''
        self.logger.info(f"Closing connection to {(self.host, self.port)}")
        try:
            self.sock.close()
        except OSError as e:
            self.logger.error(f"Error: socket.close() exception for {(self.host, self.port)}: {e!r}")
        finally:
            # Delete reference to socket object for garbage collection
            self.sock = None
        
    def _create_request(self, detections):
        ''' Sets up dict to send '''
        return dict(
            type="text/json",
            encoding="utf-8",
            content=detections
        )
        
    def send_detections(self, detections):
        ''' Send the detections over the network '''
        # Convert detection dict to encoded dictionary
        request = self._create_request(detections)
        retry_attempt = False
        success = False
        attempt_i = 0
        # while unsuccess send and attemps < MAX:
        while not success and attempt_i < self.MAX_RECONNECTION_ATTEMPTS:
            try:
                if retry_attempt:
                    self.logger.debug("About to attempt to reconnect...")
                    self.sock.connect_ex((self.host, self.port))
                    time.sleep(3)
                    self.logger.info("Reconnected to server.")

                # This will send all the None
                message = libclient.Message(self.sock, (self.host, self.port), request)
                # self.logger.debug("About to attempt to send...")
                message.write()
                success = True

            except Exception as e:
                
                # Try to reconnect
                retry_attempt = True
                attempt_i += 1
                self.logger.error(f"Error: {e}. Attempt {attempt_i}/{self.MAX_RECONNECTION_ATTEMPTS} to reconnect...")
                self.sock.close()
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.setblocking(False)
                time.sleep(3)

        if not success:
            self.logger.error('Max reconnection attempts reached. Exiting...')
            self.close()
            sys.exit()
        

if __name__ == "__main__":
    HOSTNAME = 'robo-retriever.local'

    # create self.logger
    logging.config.fileConfig('log.conf')
    logger = logging.getLogger(__name__)
    con = ConnectionClient(hostname=HOSTNAME)
    cs_stream = cs.CameraStream(src=f'tcp://{HOSTNAME}:8554').start()
    # cs_stream = cs.CameraStream(src='2024-09-28_16-01-10-validation-converted.mp4', video_file_fps=30).start()
    # cs_stream = cs.CameraStream(src='video1-converted.mp4', video_file_fps=30).start()
    res = Inference(cs_stream, model_path='tennis_court2.pt').start()
    # attempt_i = 0
    while True:
        
        detections = res.read_plot()
        # detections[-1]['line_detection'] = attempt_i
        # attempt_i += 1
        # logger.info(f'sending detections = {detections}')
        con.send_detections(detections)
        time.sleep(0.5)


