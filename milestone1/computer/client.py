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
import logging
import logging.config

# create logger
logging.config.fileConfig('log.conf')
logger = logging.getLogger(__name__)

class ConnectionClient:
    MAX_RECONNECTION_ATTEMPTS = 10
    def __init__(self, hostname='robo-retriever.local', port=65432):
        self.host = hostname
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setblocking(False)
        logger.info(f"Starting connection to {(self.host, self.port)}")
        self.sock.connect_ex((self.host, self.port))

    def close(self):
        ''' Closes the connection '''
        logger.info(f"Closing connection to {(self.host, self.port)}")
        try:
            self.sock.close()
        except OSError as e:
            logger.error(f"Error: socket.close() exception for {(self.host, self.port)}: {e!r}")
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

        except (socket.error, ConnectionResetError, UnboundLocalError) as e:
            for i in range(self.MAX_RECONNECTION_ATTEMPTS):
                try:
                    # Try to reconnect
                    logger.error(f"Error: {e}. Attempting to reconnect...")
                    sock.close()
                    time.sleep(3)  # Wait before reconnecting

                    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    self.sock.setblocking(False)
                    self.sock.connect_ex((self.host, self.port))
                    logger.info("Reconnected to server.")
                    request.write()

                except (socket.error, ConnectionResetError, UnboundLocalError) as e:
                    logger.error(f"Attempt {i + 1}/{self.MAX_RECONNECTION_ATTEMPTS} Failed to reconnect: {e}")
                    
                else:
                    break
            else:
                logger.error('Max reconnection attempts reached. Exiting...')
                raise Exception('Could not reconnect')

        

if __name__ == "__main__":
    con = ConnectionClient(hostname='robo-retriever.local')
    res = Inference().start()
    while True:
        detections = res.read_plot()
        con.send_detections(detections)
        time.sleep(0.5)


