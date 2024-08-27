import socket
import io
import picamera2

# Configuration
FRAMERATE = 30
WIDTH = 1280
HEIGHT = 720
TCP_IP = '0.0.0.0'
TCP_PORT = 8554

def start_stream():
    with picamera2.Picamera2() as camera:
        # Configure the camera
        camera.resolution = (WIDTH, HEIGHT)
        camera.framerate = FRAMERATE

        # Prepare a stream to send the video output
        stream = io.BytesIO()
        
        # Start the TCP server
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
            server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server_socket.bind((TCP_IP, TCP_PORT))
            server_socket.listen(0)

            print(f'Streaming on tcp://{TCP_IP}:{TCP_PORT}...')

            # Accept a single connection and make a file-like object out of it
            connection, client_address = server_socket.accept()
            connection = connection.makefile('wb')
            try:
                camera.start_recording(connection, 'h264')
                camera.wait_recording(0)  # Run indefinitely
            except KeyboardInterrupt:
                print("Stream interrupted, stopping...")
            finally:
                camera.stop_recording()
                connection.close()

if __name__ == "__main__":
    start_stream()
