# needs to be run in RoboRetriever python venv
from ultralytics import YOLO

# Load a pretrained YOLOv8n model
model = YOLO('yolov8n.pt')

# Run inference on the source
# results = model(source="https://www.youtube.com/watch?v=UsEarBK6ojY", show=True) 
results = model(source="tcp://192.168.4.1:8554", show=True)
