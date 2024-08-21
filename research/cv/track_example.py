from ultralytics import YOLO

# Load a model
model = YOLO("yolov8s.pt")  # load an official detection model

# Track with the model
results = model.track(source="https://www.youtube.com/watch?v=sp6LpqPOENc", show=True)
