from dotenv import load_dotenv
import inference
import supervision as sv
import cv2
import numpy as np
import matplotlib.pyplot as plt
import math

load_dotenv()

TRIGGER_OFFSET = 0.2
TRIGGER_SIZE = 0.5

# folder = "../opencv_tests/court_lines/"
# image_file_names = ['20240820_124734.jpg','20240820_124438.jpg','20240820_124447.jpg','20240820_124451.jpg','20240820_124509.jpg','20240820_124511.jpg',
#                     '20240820_124521.jpg','20240820_124524.jpg','20240820_124535.jpg','20240820_124539.jpg','20240820_124551.jpg','20240820_124554.jpg',
#                     '20240820_124603.jpg','20240820_124612.jpg','20240820_124619.jpg','20240820_124626.jpg','20240820_124701.jpg','20240820_124708.jpg',
#                     '20240820_124732.jpg','20240820_124745.jpg','20240820_124802.jpg','20240820_124804.jpg','20240820_124806.jpg','20240820_124808.jpg',
#                     '20240820_124810.jpg','20240820_124820.jpg','20240820_124821.jpg','20240820_124826.jpg','20240820_124827.jpg','20240820_124831.jpg',
#                     '20240820_124833.jpg','20240820_124835.jpg','20240820_124838.jpg']

folder = "../opencv_tests/image_data/"
image_file_names = ['img1.png','img2.png','img3.png','img4.png','img5.png','img6.png','img7.png','img8.png','img9.png','img10.png']

current_image_index = 0

image_file = folder+image_file_names[current_image_index]
image_orig = cv2.imread(image_file)
image = cv2.imread(image_file)

model = inference.get_model("roboretriver-linetest/1")
results = model.infer(image)[0]

detections = sv.Detections.from_inference(results)

# Draw bounding boxes
# for box in detections.xyxy:
#     cv2.rectangle(image, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (255,0,0), 2)

print("Mask shape:", detections.mask.shape)  # 3, 720, 1280

# Draw trigger box
height, width = image.shape[:2]
height_offset = int(height * TRIGGER_OFFSET)
box_center = (width // 2, height // 2 + height_offset)
box_size = int(min(width, height) * TRIGGER_SIZE)
box_x1 = math.floor(box_center[0] - box_size * (0.7+TRIGGER_SIZE))
box_y1 = box_center[1] - box_size // 2
box_x2 = math.floor(box_center[0] + box_size * (0.7+TRIGGER_SIZE))
box_y2 = box_center[1] + box_size // 2
cv2.rectangle(image, (box_x1, box_y1), (box_x2, box_y2), (0,0,255), 2)

# Does it cross the trigger box
triggered = False
angle = None
distance = None
line_vector = np.array([0, 0])

for mask in detections.mask:
    binary_mask = (mask.astype(np.uint8)) * 255
    contours, _ = cv2.findContours(binary_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(image, contours, -1, (0, 255, 0), 2)
    cv2.fillPoly(image, contours, (0, 255, 0))

    trigger_box = np.array([[box_x1, box_y1], [box_x2, box_y1], [box_x2, box_y2], [box_x1, box_y2]])
    mask_points_inside_trigger = []

    for contour in contours:
        for point in contour:
            x, y = point[0]
            if box_x1 <= x <= box_x2 and box_y1 <= y <= box_y2:
                mask_points_inside_trigger.append([x, y])

    if mask_points_inside_trigger:
        mask_points_inside_trigger = np.array(mask_points_inside_trigger)
        [vx, vy, x0, y0] = cv2.fitLine(mask_points_inside_trigger, cv2.DIST_L2, 0, 0.01, 0.01)
        a = vy
        b = -vx

        print([True, np.array([a,b])])

        x1 = int(x0 - 1000 * vx)  # Extend the line for visualization
        y1 = int(y0 - 1000 * vy)
        x2 = int(x0 + 1000 * vx)
        y2 = int(y0 + 1000 * vy)
        cv2.line(image, (x1, y1), (x2, y2), (255, 0, 0), 2)
    else:
        print([False, (None,None)])




#cv2.imshow('Original image', image_orig)
#cv2.imshow('Bounding Boxes', image)

bounding_box_annotator = sv.PolygonAnnotator()
label_annotator = sv.LabelAnnotator()

annotated_image = bounding_box_annotator.annotate(scene=image, detections=detections)
annotated_image = label_annotator.annotate(scene=annotated_image, detections=detections)

sv.plot_image(annotated_image)
