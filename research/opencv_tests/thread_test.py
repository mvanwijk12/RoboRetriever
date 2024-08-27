import cv2
import threading
from main import detect_line

# Global variable to store the current frame
frame = None

# Thread event to stop the threads gracefully
stop_event = threading.Event()

def camera_capture():
    global frame
    cap = cv2.VideoCapture(1)

    while not stop_event.is_set():
        ret, new_frame = cap.read()
        if ret:
            frame = new_frame

    cap.release()

def display_thread_1():
    while not stop_event.is_set():
        if frame is not None:
            print(detect_line(frame, True))

            if cv2.waitKey(1) & 0xFF == ord('q'):
                stop_event.set()

def display_thread_2():
    while not stop_event.is_set():
        if frame is not None:
            cv2.imshow("Thread 2", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                stop_event.set()

# Start the camera capture thread
capture_thread = threading.Thread(target=camera_capture)
capture_thread.start()

# Start the display threads
thread_1 = threading.Thread(target=display_thread_1)
thread_2 = threading.Thread(target=display_thread_2)

thread_1.start()
thread_2.start()

# Wait for all threads to finish
capture_thread.join()
thread_1.join()
thread_2.join()

cv2.destroyAllWindows()
