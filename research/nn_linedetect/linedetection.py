from ultralytics import YOLO
import cv2
import numpy as np
import math

TRIGGER_OFFSET = 0.2
TRIGGER_SIZE = 0.5

# folder = "../opencv_tests/court_lines/"
# image_file_names = ['20240820_124734.jpg','20240820_124438.jpg','20240820_124447.jpg','20240820_124451.jpg','20240820_124509.jpg','20240820_124511.jpg',
#                     '20240820_124521.jpg','20240820_124524.jpg','20240820_124535.jpg','20240820_124539.jpg','20240820_124551.jpg','20240820_124554.jpg',
#                     '20240820_124603.jpg','20240820_124612.jpg','20240820_124619.jpg','20240820_124626.jpg','20240820_124701.jpg','20240820_124708.jpg',
#                     '20240820_124732.jpg','20240820_124745.jpg','20240820_124802.jpg','20240820_124804.jpg','20240820_124806.jpg','20240820_124808.jpg',
#                     '20240820_124810.jpg','20240820_124820.jpg','20240820_124821.jpg','20240820_124826.jpg','20240820_124827.jpg','20240820_124831.jpg',
#                     '20240820_124833.jpg','20240820_124835.jpg','20240820_124838.jpg']

# folder = "../opencv_tests/image_data/"
# image_file_names = ['img1.png','img2.png','img3.png','img4.png','img5.png','img6.png','img7.png','img8.png','img9.png','img10.png']

folder = "../opencv_tests/new_images/"
image_file_names = ['image_1-2024-09-13_14-08-15.jpg','image_2-2024-09-13_14-08-15.jpg','image_3-2024-09-13_14-08-15.jpg','image_4-2024-09-13_14-08-15.jpg',
                    'image_5-2024-09-13_14-08-15.jpg','image_6-2024-09-13_14-08-15.jpg','image_7-2024-09-13_14-08-15.jpg','image_8-2024-09-13_14-08-15.jpg',
                    'image_9-2024-09-13_14-08-15.jpg','image_10-2024-09-13_14-08-15.jpg','image_11-2024-09-13_14-08-15.jpg','image_12-2024-09-13_14-08-15.jpg',
                    'image_13-2024-09-13_14-08-15.jpg','image_14-2024-09-13_14-08-15.jpg','image_15-2024-09-13_14-08-15.jpg','image_16-2024-09-13_14-08-15.jpg',
                    'image_17-2024-09-13_14-08-15.jpg','image_18-2024-09-13_14-08-15.jpg','image_19-2024-09-13_14-08-15.jpg','image_20-2024-09-13_14-08-15.jpg']

current_image_index = 0 # 19

image_file = folder+image_file_names[current_image_index]
image_orig = cv2.imread(image_file)
image = cv2.imread(image_file)

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


def detect_line(masks):
    # Function to fit and return a line equation from a single mask
    def calc_single_line(mask):
        indices = np.argwhere(mask == 1)  # Just get the 1s
        x_coords = indices[:, 1]
        y_coords = indices[:, 0]

        # Least squares method
        A = np.vstack([x_coords, np.ones(len(x_coords))]).T
        a, c = np.linalg.lstsq(A, y_coords, rcond=None)[0]

        return np.array([a,1])

    for line in masks:
        line_mask = np.array(line.data[0])  # 3D array of 1
        mask_con = np.sum(line_mask==0)
        mask_pro = np.sum(line_mask==1)
        print("Mask coverage: ", mask_pro/(mask_con+mask_pro))
        print(calc_single_line(line_mask))
        print("\n")


model = YOLO('best.pt')
detections = model.predict([image_file], classes=1)  # class 1 is line

for result in detections:
    masks = result.masks
    #print(masks.shape)
    detect_line(masks)

    result.show()
