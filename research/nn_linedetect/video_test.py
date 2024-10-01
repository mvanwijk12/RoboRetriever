from ultralytics import YOLO
import numpy as np
import cv2
import logging
import logging.config
from linedetection import LineDetector


def process_video(video_path, model, detector):
    cap = cv2.VideoCapture(video_path)

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        frame_height, frame_width, _ = frame.shape
        detections = model.predict(frame, classes=1)
        result = detections[0]
        masks = result.masks

        if masks is not None:
            triggered, line, all_lines = detector.detect(masks, orig_img=frame)
            print(triggered, line)

        cv2.imshow("Lines", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    logger = logging.getLogger(__name__)

    video_path = "../opencv_tests/videos/2024-09-07_00-49-34-validation-converted.mp4"
    model = YOLO('best.pt')
    detector = LineDetector(viz_type=1)
    
    process_video(video_path, model, detector)
