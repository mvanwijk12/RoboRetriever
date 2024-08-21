from ultralytics import YOLO
import cv2

# # Load a YOLOv8 model from a pre-trained weights file
# model = YOLO("yolov8n.pt")

# # Run MODE mode using the custom arguments ARGS (guess TASK)
img = cv2.imread('test_img.jpeg')
# results = model.predict(source=img, save=True, save_txt=True)
# print(results)


# Load a pretrained YOLO model
model = YOLO("yolov8n.pt")

# Perform object detection on an image
# results = model("https://ultralytics.com/images/bus.jpg")
results = model(img)

# Print results
print(results[0].probs)

# Visualize the results
# for result in results:
#     result.show()