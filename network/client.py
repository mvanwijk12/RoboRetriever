#!/usr/bin/env python3
import selectors
import socket
import sys
import traceback
import libclient
import numpy as np
import time

class Connection:
    def __init__(self, hostname='127.0.0.1', port=65432):
        self.host = hostname
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setblocking(False)
        print(f"Starting connection to {(self.host, self.port)}")
        self.sock.connect_ex((self.host, self.port))
        self.MAX_RECONNECTION = 10

    def close(self):
        ''' Closes the connection '''
        pass
        
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
        
        try:
            # This will send all the 
            request.write()

        except (socket.error, ConnectionResetError) as e:
            for i in range(self.MAX_RECONNECTION):
                try:
                    # Try to reconnect
                    print(f"Error sending data: {e}. Attempting to reconnect...")
                    sock.close()
                    time.sleep(3)  # Wait before reconnecting
                    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    self.sock.setblocking(False)
                    self.sock.connect_ex((self.host, self.port))
                    print("Reconnected to server.")
                    request.write()

                except socket.error as e:
                    print(f"Attempt {i + 1}/{self.MAX_RECONNECTION} Failed to reconnect: {e}")
                    
                else:
                    break
        
        #finally:
            #sel.close()
        

if __name__ == "__main__":
    sel = selectors.DefaultSelector()
    # cap = Inference().start()
    while True:
        detections = dict(
                    xyxy=np.array([[1006.654, 653.64, 1038.28, 684.96]]).tolist(),
                    confidence=np.array([0.89]).tolist(),
                    class_id=np.array([32]).tolist(),
                    tracker_id=np.array([3]).tolist(),
                    data={'class_name': np.array(['sports ball'], dtype='<U13').tolist()}
                )
        send_detections(detections)
        time.sleep(0.5)


# detections = dict(
#                 xyxy=np.array([[1006.654, 653.64, 1038.28, 684.96]]).tolist(),
#                 confidence=np.array([0.89]).tolist(),
#                 class_id=np.array([32]).tolist(),
#                 tracker_id=np.array([3]).tolist(),
#                 data={'class_name': np.array(['sports ball'], dtype='<U13').tolist()}
# )
# detections = dict(
#                 xyxy=np.array([[]]).tolist(),
#                 confidence=np.array([]).tolist(),
#                 class_id=np.array([]).tolist(),
#                 tracker_id=np.array([]).tolist(),
#                 data=None
# )
