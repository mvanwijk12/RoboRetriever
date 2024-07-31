# run this program on the Mac to display image streams from multiple RPis
import cv2
import imagezmq
from ultralytics import YOLO
import numpy as np


model = YOLO("yolov8n.pt")
image_hub = imagezmq.ImageHub()

while True:  # show streamed images until Ctrl-C
    rpi_name, image = image_hub.recv_image()
    # print(np.shape(image))
    results = model(image)
    for result in results:
        result.show()

    # cv2.imshow(rpi_name, image) # 1 window for each RPi
    # cv2.waitKey(5000)
    image_hub.send_reply(b'OK')