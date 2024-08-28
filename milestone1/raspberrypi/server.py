"""
This is a python class for creating a server

Modified from: https://realpython.com/python-sockets/ 
"""
__author__ = "Matt van Wijk"
__date__ = "28/08/2024"

import selectors
import socket
import traceback
import libserver
from threading import Thread, Condition
import logging
import logging.config

# create logger
logging.config.fileConfig('log.conf')
server_logger = logging.getLogger('server')

class ConnectionServer:
    def __init__(self, host='', port= 65432):
        self.host = host
        self.port = port
        self.msg = None
        self.msg_ready = False
        self.condition = Condition()
        self.sel = selectors.DefaultSelector()
        self.lsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # Avoid bind() exception: OSError: [Errno 48] Address already in use
        self.lsock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.lsock.bind((host, port))
        self.lsock.listen()
        server_logger.info(f"Listening on {(host, port)}")
        self.lsock.setblocking(False)
        self.sel.register(self.lsock, selectors.EVENT_READ, data=None)

    def start(self):
        self.stop = False
        Thread(target=self.message_update, args=()).start()
        return self

    def message_update(self):
        try:
            while not self.stop:
                events = self.sel.select(timeout=1)
                for key, mask in events:
                    if key.data is None: # Initial
                        self.accept_wrapper(key.fileobj)
                    else:
                        message = key.data
                        try:
                            self.msg = message.process_events(mask)
                        
                            with self.condition:
                                self.msg_ready = True
                                self.condition.notify_all()
                        except Exception:
                            server_logger.error(
                                f"Main: Error: Exception for {message.addr}:\n"
                                f"{traceback.format_exc()}"
                            )
                            message.close()
    
        except (KeyboardInterrupt, RuntimeError):
            server_logger.info("Caught keyboard/RuntimeError interrupt, exiting")
        finally:
            self.sel.close()

    def get_message(self):
        try:
            success = False
            while not success and not self.msg_ready:
                with self.condition:
                    success = self.condition.wait(timeout=1)
        except: # catch all exceptions including KeyboardInterrupt
            server_logger.error(f"Main: Error: Exception for {traceback.format_exc()}")
            self.close()
            raise Exception
        else:
            server_logger.debug('new msg received')
            self.msg_ready = False 
            return self.msg

    def close(self):
        self.stop = True

    def accept_wrapper(self, sock):
        conn, addr = sock.accept()  # Should be ready to read
        server_logger.info(f"Accepted connection from {addr}")
        conn.setblocking(False)
        message = libserver.Message(self.sel, conn, addr)
        self.sel.register(conn, selectors.EVENT_READ, data=message)



if __name__ == "__main__":
    con = ConnectionServer(host='').start()
    # try:
    while True:
        x = con.get_message()
        if x is not None:
            server_logger.info(f"ERROR IS {x['error']}")
        else:
            server_logger.info("NO DETECTIONS")
