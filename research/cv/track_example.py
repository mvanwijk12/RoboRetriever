from ultralytics import YOLO
import supervision as sv
import matplotlib.pyplot as plt
import numpy as np
import cv2

# Load a model
# model = YOLO("yolov8s.pt")  # load an official detection model
model = YOLO("../../models/yolov8s.pt")  # load an official detection model


# Track with the model
result = model.track(source="../../img/test_img.jpeg")[0]
desired_class_ids = [32]

# Convert to sv detection object
detections = sv.Detections.from_ultralytics(result)
if result.boxes.id is not None:
    detections.tracker_id = result.boxes.id.cpu().numpy().astype(int) # TODO: replace with cuda instead of cpu()?

mask = np.isin(detections.class_id, desired_class_ids)

# Step 2: Apply the mask to filter the detections
filtered_detections = sv.Detections(
    xyxy=detections.xyxy[mask],
    confidence=detections.confidence[mask],
    class_id=detections.class_id[mask],
    tracker_id=detections.tracker_id[mask] if detections.tracker_id is not None else None,
    data={key: value[mask] for key, value in detections.data.items()} if detections.data else None
)

# Filter detections to only keep desired classes
# filtered_detections = [d for d in detections if d.class_id in desired_class_ids]
img_frame = result.orig_img
#img_frame = cv2.cvtColor(img_frame, cv2.COLOR_BGR2RGB)

# print("The length is ", len(results))
# print("Result is ", results[0].boxes.cls.cpu().numpy().astype(int))

box_annotator = sv.BoxAnnotator()  # Create a BoxAnnotator
annotated_image = box_annotator.annotate(scene=img_frame.copy(), detections=filtered_detections)

# Step 4: Plot the image with bounding boxes
# plt.figure(figsize=(10, 10))
# plt.imshow(annotated_image)
# plt.axis('off')  # Hide axes
# plt.show()

cv2.imshow('frame', annotated_image)
cv2.waitKey(2000)