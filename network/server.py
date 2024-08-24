#!/usr/bin/env python3

import selectors
import socket
import traceback
import libserver
from threading import Thread, Condition

# sel = selectors.DefaultSelector()

class ConnectionServer:
    def __init__(self, host='127.0.0.1', port= 65432):
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
        print(f"Listening on {(host, port)}")
        self.lsock.setblocking(False)
        self.sel.register(self.lsock, selectors.EVENT_READ, data=None)

    def start(self):
        self.stop = False
        Thread(target=self.message_update, args=()).start()
        return self

    def message_update(self):
        try:
            while True:
                if self.stop: return
                events = self.sel.select(timeout=None)
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
                            print(
                                f"Main: Error: Exception for {message.addr}:\n"
                                f"{traceback.format_exc()}"
                            )
                            message.close()
    
        except (KeyboardInterrupt, RuntimeError):
            print("Caught keyboard interrupt, exiting")
        finally:
            self.sel.close()

    def get_message(self):
        if not self.msg_ready:
            with self.condition:
                self.condition.wait()

        self.msg_ready = False 
        return self.msg

    def close(self):
        self.stop = True

    def accept_wrapper(self, sock):
        conn, addr = sock.accept()  # Should be ready to read
        print(f"Accepted connection from {addr}")
        conn.setblocking(False)
        message = libserver.Message(self.sel, conn, addr)
        self.sel.register(conn, selectors.EVENT_READ, data=message)



if __name__ == "__main__":
    con = ConnectionServer(host='').start()
    try:
        while True:
            x = con.get_message()
            if x is not None:
                print(f"ERROR IS {x['error']}")
            else:
                print("NO DETECTIONS")
    except KeyboardInterrupt:
        con.close()
        print('closing..')

# if len(sys.argv) != 3:
#     print(f"Usage: {sys.argv[0]} <host> <port>")
#     sys.exit(1)

# host, port = sys.argv[1], int(sys.argv[2])
# host, port = '127.0.0.1', 65432
# lsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# # Avoid bind() exception: OSError: [Errno 48] Address already in use
# lsock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
# lsock.bind((host, port))
# lsock.listen()
# print(f"Listening on {(host, port)}")
# lsock.setblocking(False)
# sel.register(lsock, selectors.EVENT_READ, data=None)

# try:
#     while True:
#         events = sel.select(timeout=None)
#         for key, mask in events:
#             if key.data is None: # Initial
#                 accept_wrapper(key.fileobj)
#             else:
#                 message = key.data
#                 try:
#                     message.process_events(mask)
#                 except Exception:
#                     print(
#                         f"Main: Error: Exception for {message.addr}:\n"
#                         f"{traceback.format_exc()}"
#                     )
#                     message.close()
# except KeyboardInterrupt:
#     print("Caught keyboard interrupt, exiting")
# finally:
#     sel.close()
