from ultralytics import YOLO
import numpy as np

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

current_image_index = 19 # 19
image_file = folder+image_file_names[current_image_index]


def detect_line(masks):
    # Function to fit and return a line equation from a single mask
    def calc_single_line(mask):
        indices = np.argwhere(mask == 1)  # Just get the 1s
        x_coords = indices[:, 1]  # Rows
        y_coords = indices[:, 0]  # Columns

        # Least squares method
        A = np.vstack([x_coords, np.ones(len(x_coords))]).T
        a, c = np.linalg.lstsq(A, y_coords, rcond=None)[0]

        return np.array([a,1])

    # Function to find if a mask crosses the trigger box
    def calc_trigger(mask, box_width_ratio=0.8, box_height_ratio=0.25, bottom_offset_ratio=0.1):
        frame_height, frame_width = mask.shape
        box_width = int(box_width_ratio * frame_width)
        box_height = int(box_height_ratio * frame_height)
        box_x_start = (frame_width - box_width) // 2
        box_y_start = int(frame_height * (1 - bottom_offset_ratio - box_height_ratio)) 

        indices = np.argwhere(mask == 1)
        line_in_box = [
            [x, y] for y, x in indices
            if box_x_start <= x <= box_x_start + box_width and box_y_start <= y <= box_y_start + box_height
        ]

        if line_in_box:
            return True
        else:
            return False
        
    # Function to calculate the mask coverage across the frame
    def calc_cover(mask):
        mask_con = np.sum(line_mask==0)
        mask_pro = np.sum(line_mask==1)
        return "Mask coverage: " + str((mask_pro/(mask_con+mask_pro))*100) + "%"

    arr_out = []
    triggered = False
    for line in masks:
        line_mask = np.array(line.data[0])  # 3D array of 1
        #print(calc_cover(line_mask))

        # TODO: return only the main line (biggest in box)
        if calc_trigger(line_mask):
            # if the line is in the trigger box
            triggered = True
            arr_out.append(calc_single_line(line_mask))
    
    return triggered, None, None, arr_out


if __name__ == "__main__":
    model = YOLO('best.pt')
    detections = model.predict([image_file], classes=1)  # class 1 is line

    for result in detections:
        masks = result.masks
        #print(masks.shape)
        print(detect_line(masks))

        result.show()
