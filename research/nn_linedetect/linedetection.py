from ultralytics import YOLO
import numpy as np
import cv2
import logging
import logging.config

def detect_line(masks, box_width_ratio=0.9, box_height_ratio=0.25, bottom_offset_ratio=0.05, orig_img=None):
    # Initialise the trigger box
    frame_height, frame_width = masks[0].data[0].shape
    box_width = int(box_width_ratio * frame_width)
    box_height = int(box_height_ratio * frame_height)
    box_x_start = (frame_width - box_width) // 2
    box_y_start = int(frame_height * (1 - bottom_offset_ratio - box_height_ratio)) 


    # Function to fit and return a line equation from a single mask
    def calc_single_line(mask):
        indices = np.argwhere(mask == 1)  # Just get the 1s
        x_coords = indices[:, 1]  # Cols
        y_coords = indices[:, 0]  # Rows

        # Least squares method to get y=mx+c
        A = np.vstack([x_coords, np.ones(len(x_coords))]).T
        a, c = np.linalg.lstsq(A, y_coords, rcond=None)[0]

        # Clip the line to the frame for visualisation
        y1 = int(a * 0 + c)
        x1 = 0
        y2 = int(a * frame_width + c)
        x2 = frame_width
        y1 = np.clip(y1, 0, frame_height)
        y2 = np.clip(y2, 0, frame_height)

        return [np.array([a,1]), (x1, y1, x2, y2)]

    # Function to find if a mask crosses the trigger box and by how much
    def calc_trigger(mask):
        indices = np.argwhere(mask == 1)  # coords of 1s
        line_in_box = [
            [x, y] for y, x in indices
            if box_x_start <= x <= box_x_start + box_width and box_y_start <= y <= box_y_start + box_height
        ]

        if line_in_box:
            trigger_box = mask[box_y_start:box_y_start + box_height, box_x_start:box_x_start + box_width]
            trig_pro = np.sum(trigger_box == 1)
            return trig_pro
        else:
            return None
        
    # Function to calculate the mask coverage across the frame
    def calc_cover_frame(mask):
        mask_con = np.sum(mask==0)
        mask_pro = np.sum(mask==1)
        return "Mask coverage: " + str((mask_pro/(mask_con+mask_pro))*100) + "%"

    trig_lines = []
    triggered = False
    largest_line = [None, None]
    x_lines = (None, None, None, None)

    # Perform visualisation if given the original image
    if orig_img is not None:
        # Add a trigger box
        top_left = (box_x_start, box_y_start)
        bottom_right = (box_x_start + box_width, box_y_start + box_height)
        img_resize = cv2.resize(orig_img, (frame_width, frame_height))
        trig_viz = cv2.rectangle(img_resize, top_left, bottom_right, 255, 2)

    # Go through all lines
    for line in masks:
        line_mask = np.array(line.data[0])  # 3D array of 1
        #print(calc_cover_frame(line_mask))
        trig_amount = calc_trigger(line_mask)
        single_line, x_lines = calc_single_line(line_mask)

        # For viz
        line_col = (0, 255, 0)
        line_thc = 1

        if trig_amount is not None:
            # if the line is in the trigger box
            triggered = True
            trig_lines.append((single_line, trig_amount))
            largest_line = max(trig_lines, key=lambda x: x[1])

            # For viz
            line_col = (0, 0, 255)
            line_thc = 2

        # Add line as visualisation
        if orig_img is not None:
            x1, y1, x2, y2 = x_lines                
            line_viz = cv2.line(trig_viz, (x1, y1), (x2, y2), line_col, line_thc)
            cv2.imshow("Detected Lines with Trigger Box", line_viz)
    
    return triggered, largest_line[0]  # formerly trig (bin), angle, distance, np array of a and b


if __name__ == "__main__":
    #logging.config.fileConfig('log.conf')
    logger = logging.getLogger(__name__)

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

    model = YOLO('best.pt')
    detections = model.predict([image_file], classes=1)  # class 1 is line
    result = detections[0]
    masks = result.masks

    visualise = True

    if visualise:
        img = result.plot()
        print(detect_line(masks, orig_img=img))
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    else:
        print(detect_line(masks))
