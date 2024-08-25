import socket
import io
from picamera2 import Picamera2
import time
import threading

# Configuration
FRAMERATE = 30
WIDTH = 1280
HEIGHT = 720
TCP_IP = '0.0.0.0'
TCP_PORT = 8554

def stream_video(camera, connection):
    try:
        # Create a BytesIO stream to capture video
        stream = io.BytesIO()
        camera.start_recording(stream, 'h264')

        while True:
            # Reset the stream position
            stream.seek(0)
            # Read the stream contents
            connection.write(stream.read())
            # Flush the connection to send data immediately
            connection.flush()
            # Reset the stream for the next frame
            stream.seek(0)
            stream.truncate()
            
            # Wait for the next frame
            time.sleep(1 / FRAMERATE)
    except Exception as e:
        print(f"Error occurred: {e}")
    finally:
        camera.stop_recording()
        connection.close()

def start_stream():
    # Initialize the camera
    picam2 = Picamera2()
    config = picam2.create_video_configuration(main={"size": (WIDTH, HEIGHT)})
    picam2.configure(config)

    # Start the camera preview
    picam2.start_preview()
    
    # Start the TCP server
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.bind((TCP_IP, TCP_PORT))
        server_socket.listen(0)

        print(f'Streaming on tcp://{TCP_IP}:{TCP_PORT}...')

        # Accept a single connection
        connection, client_address = server_socket.accept()

        try:
            # Start the video streaming in a separate thread
            stream_thread = threading.Thread(target=stream_video, args=(picam2, connection))
            stream_thread.start()
            stream_thread.join()
        except KeyboardInterrupt:
            print("Streaming stopped by user")
        finally:
            connection.close()

if __name__ == "__main__":
    start_stream()
