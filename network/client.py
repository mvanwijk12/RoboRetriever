#!/usr/bin/env python3
import selectors
import socket
import sys
import traceback
import libclient
import numpy as np
import time

class Connection:
    def __init__(self):
        pass

    def start(self):
        ''' Starts the connection '''
        pass

    def close(self):
        ''' Closes the connection '''
        pass
        
    def create_request(detections):
        ''' Sets up dict to send '''
        return dict(
            type="text/json",
            encoding="utf-8",
            content=detections
        )

    def start_connection(HOST, PORT, request):
        ''' Starts up connection with HOST '''
        addr = (HOST, PORT)
        print(f"Starting connection to {addr}")
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setblocking(False)
        sock.connect_ex(addr)
        events = selectors.EVENT_READ | selectors.EVENT_WRITE
        message = libclient.Message(sel, sock, addr, request)
        sel.register(sock, events, data=message)

    def send_detections(detections):
        HOST, PORT = '127.0.0.1', 65432
        request = create_request(detections)
        start_connection(HOST, PORT, request)

        try:
            while True:
                events = sel.select(timeout=1)
                for key, mask in events:
                    message = key.data
                    try:
                        message.process_events(mask)
                    except Exception:
                        print(
                            f"Main: Error: Exception for {message.addr}:\n"
                            f"{traceback.format_exc()}"
                        )
                        message.close()
                # Check for a socket being monitored to continue.
                if not sel.get_map():
                    print('breaking out of while True loop...')
                    break
        except KeyboardInterrupt:
            print("Caught keyboard interrupt, exiting")
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
