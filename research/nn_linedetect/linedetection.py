from ultralytics import YOLO
import numpy as np
import cv2
import logging
import logging.config

class LineDetector:
    def __init__(self, box_width_ratio=0.9, box_height_ratio=0.25, bottom_offset_ratio=0.05, viz_type=1):
        self.box_width_ratio = box_width_ratio
        self.box_height_ratio = box_height_ratio
        self.bottom_offset_ratio = bottom_offset_ratio
        self.viz_type = viz_type
        self.box_width = None
        self.box_height = None
        self.box_x_start = None
        self.box_y_start = None
    
    def _calc_single_line(self, mask, frame_height):
        indices = np.argwhere(mask == 1)  # Just get the 1s
        x_coords = indices[:, 1]  # Cols
        y_coords = indices[:, 0]  # Rows

        # Least squares method to get y=mx+c
        A = np.vstack([x_coords, np.ones(len(x_coords))]).T
        a, c = np.linalg.lstsq(A, y_coords, rcond=None)[0]

        # Clip the line to the frame for visualisation
        x1 = np.min(x_coords)
        x2 = np.max(x_coords)
        y1 = np.clip(int(a * x1 + c), 0, frame_height)
        y2 = np.clip(int(a * x2 + c), 0, frame_height)

        return [np.array([a, 1]), (x1, y1, x2, y2)]
    
    def _calc_trigger(self, mask):
        indices = np.argwhere(mask == 1)  # coords of 1s
        line_in_box = [
            [x, y] for y, x in indices
            if self.box_x_start <= x <= self.box_x_start + self.box_width and self.box_y_start <= y <= self.box_y_start + self.box_height
        ]

        if line_in_box:
            trigger_box = mask[self.box_y_start:self.box_y_start + self.box_height, self.box_x_start:self.box_x_start + self.box_width]
            trig_pro = np.sum(trigger_box == 1)
            return trig_pro
        else:
            return None

    def _calc_cover_frame(self, mask):
        mask_con = np.sum(mask == 0)
        mask_pro = np.sum(mask == 1)
        return "Mask coverage: " + str((mask_pro / (mask_con + mask_pro)) * 100) + "%"

    def detect(self, masks, orig_img=None):
        # Initialise the trigger box
        frame_height, frame_width = masks[0].data[0].shape
        self.box_width = int(self.box_width_ratio * frame_width)
        self.box_height = int(self.box_height_ratio * frame_height)
        self.box_x_start = (frame_width - self.box_width) // 2
        self.box_y_start = int(frame_height * (1 - self.bottom_offset_ratio - self.box_height_ratio))

        trig_lines = []
        triggered = False
        largest_line = [None, None]
        x_lines = (None, None, None, None)

        # Perform visualisation if given the original image
        if orig_img is not None:
            img_resize = cv2.resize(orig_img, (frame_width, frame_height))
            if self.viz_type == 1:
                # Add a trigger box
                top_left = (self.box_x_start, self.box_y_start)
                bottom_right = (self.box_x_start + self.box_width, self.box_y_start + self.box_height)
                trig_viz = cv2.rectangle(img_resize, top_left, bottom_right, 255, 2)

        # Go through all lines
        all_lines = []
        for line in masks:
            line_mask = np.array(line.data[0])  # 3D array of 1
            trig_amount = self._calc_trigger(line_mask)
            single_line, x_lines = self._calc_single_line(line_mask, frame_height)
            all_lines.append(x_lines)

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
                if self.viz_type == 1:
                    line_viz = cv2.line(trig_viz, (x1, y1), (x2, y2), line_col, line_thc)
                else:
                    line_viz = cv2.line(img_resize, (x1, y1), (x2, y2), line_col, line_thc)
                cv2.imshow("Detected Lines with Trigger Box", line_viz)

        return triggered, largest_line[0], all_lines  # formerly trig (bin), angle, distance, np array of a and b


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
        detector = LineDetector(viz_type=1)
        img = result.plot()  # visualise with the bounding boxes and masks
        #img = cv2.imread(image_file)  # visualise with only the image
        print(detector.detect(masks, orig_img=img))
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    else:
        detector = LineDetector(viz_type=0)
        print(detector.detect(masks))
