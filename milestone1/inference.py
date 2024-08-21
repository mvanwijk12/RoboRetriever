"""
This is a python class to control image inference.
"""

__author__ = "Matt van Wijk"
__date__ = "20/08/2024"

from ultralytics import YOLO

class Inference:
    def __init__(self, src='tcp://robo-retriever.local:8554', frame_h=720, frame_w=1280):
        """ Initialises the camera stream object """
        pass
    def process_image(self, conf=0.5):
        pass


# Ref. https://github.com/ultralytics/ultralytics/issues/10315
# Before the loop, define the class IDs you want to keep
desired_class_ids = [0, 1, 2]  # Assuming these are the IDs for "No Helmet", "Person", "Rider"

while True:
    ret, frame = cap.read()

    for result in model.track(source=frame, stream=True, persist=True, conf=0.5):
        frame = result.orig_img
        detections = sv.Detections.from_ultralytics(result)

        if result.boxes.id is not None:
            detections.tracker_id = result.boxes.id.cpu().numpy().astype(int)

        # Filter detections to only keep desired classes
        filtered_detections = filter(lambda d: d.class_id in desired_class_ids, detections)

        labels = []
        if detections.tracker_id is not None:
            labels = [
                f"#{tracker_id} {model_config.names[class_id]} {confidence:0.2f}"
                for class_id, confidence, tracker_id
                in zip(filtered_detections.class_id, filtered_detections.confidence, filtered_detections.tracker_id)
            ]

        if labels:
            frame = box_annotator.annotate(
                scene=frame.copy(),
                detections=filtered_detections  # Use filtered detections
            )
            frame = label_annotator.annotate(
                scene=frame.copy(),
                detections=filtered_detections,  # Use filtered detections
                labels=labels
            )
        else:
            print("No Labels")

        mask = zone.trigger(detections=filtered_detections)  # Use filtered detections
        frame = zone_annotator.annotate(scene=frame.copy())

    cv2.imshow("yolov8", frame)
    if (cv2.waitKey(20) == 27):
        break

cap.release()
cv2.destroyAllWindows()












if __name__ == "__main__":
    cap = WebcamStream().start()
    while(True):
        frame = cap.read()
        cv2.imshow('frame', frame)
        cv2.waitKey(1)