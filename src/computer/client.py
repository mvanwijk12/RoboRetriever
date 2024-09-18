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
        message = libclient.Message(self.sock, (self.host, self.port), request)
        
        try:
            # This will send all the None
            message.write()

        except Exception as e:
            for i in range(self.MAX_RECONNECTION_ATTEMPTS):
                try:
                    # Try to reconnect
                    self.logger.error(f"Error: {e}. Attempting to reconnect...")
                    sock.close()

                    start_time = time.time()
                    while time.time() - start_time < 3: # Wait before reconnecting
                        pass  

                    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    self.sock.setblocking(False)
                    self.sock.connect_ex((self.host, self.port))
                    self.logger.info("Reconnected to server.")
                    message.write()

                except Exception as e:
                    self.logger.error(f"Attempt {i + 1}/{self.MAX_RECONNECTION_ATTEMPTS} Failed to reconnect: {e}")
                    
                else:
                    break
            else:
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
    res = Inference(cs_stream).start()
    trag = Tragectory(cs_stream).start()
    while True:
        trag.read_reflect_trag()
        detections = res.read_plot()
        con.send_detections(detections)
        time.sleep(0.5)


