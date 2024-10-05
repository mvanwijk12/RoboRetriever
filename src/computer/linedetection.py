from ultralytics import YOLO
import numpy as np
import cv2
import logging
import logging.config
from sklearn.linear_model import RANSACRegressor
import torch
import warnings
warnings.filterwarnings("ignore", category=UserWarning) # Suppress UserWarning from PyTorch
# import liblinedetect as lld

__author__ = "John Bui"
__modified__ = "Matt van Wijk"
__last_modified__ = "05/10/2024"

class LineDetector:
    def __init__(self, box_width_ratio=0.5, box_height_ratio=0.15, bottom_offset_ratio=0.03, masks_shape=(384, 640)):
        """ Initialises the LineDetector object
         
        Parameters:
        - box_width_ratio: specifies the width ratio of the trigger box
        - box_height_ratio: specifies the height ratio of the trigger box
        - bottom_offset_ratio: speifies the offset ratio of the trigger box from the bottom of the frame
        - masks_shape: the (height, width) of the court line masks
        
        Returns:
        - None
        """
        self.box_width_ratio = box_width_ratio
        self.box_height_ratio = box_height_ratio
        self.bottom_offset_ratio = bottom_offset_ratio

        # Initialise the trigger box
        self.frame_height, self.frame_width = masks_shape
        self.box_width = int(self.box_width_ratio * self.frame_width)
        self.box_height = int(self.box_height_ratio * self.frame_height)
        self.box_x_start = (self.frame_width - self.box_width) // 2
        self.box_y_start = int(self.frame_height * (1 - self.bottom_offset_ratio - self.box_height_ratio))

        # Trigger box
        self.top_left = (self.box_x_start, self.box_y_start)
        self.bottom_right = (self.box_x_start + self.box_width, self.box_y_start + self.box_height)
    
    def _calc_single_line(self, mask, method="least_squares"):
        """ Performs linear regression of a single mask of a court line.
         
        Parameters:
        - mask: numpy or tensor object containing the segmentation mask of a court line
        - method: either the string "least_squares" or "ransac", specifies the method of linear regression 
        
        Returns:
        - [direction, (x1, y1, x2, y2)]:
            - direction is a list of the form [a, b] where a*x + b*y = 0, specifying the direction of the court line
            - (x1, y1, x2, y2) specifies the coordinates of the end points of the linear regression line
        """
        indices = torch.argwhere(mask == 1)  # Just get the 1s
        indices = torch.tensor(indices, dtype=float)
        x_coords = indices[:, 1]  # Cols
        y_coords = indices[:, 0]  # Rows

        if method == "least_squares":
            # Least squares method to get y=mx+c
            one_tensor = torch.ones(len(x_coords)).to('cuda')
            A = torch.vstack([x_coords, one_tensor]).T
            a, c = torch.linalg.lstsq(A, y_coords, rcond=None)[0]

        elif method == "ransac":
            # RANSAC method to get y=mx+c
            x_coords_np = x_coords.cpu().numpy()
            y_coords_np = y_coords.cpu().numpy()
            ransac = RANSACRegressor(residual_threshold=5, max_trials=5)
            ransac.fit(x_coords_np.reshape(-1, 1), y_coords_np)
            a = torch.tensor(ransac.estimator_.coef_[0])  # Slope
            c = torch.tensor(ransac.estimator_.intercept_)  # Intercept

            # # Fit RANSAC model
            # mad = lld.mad_torch(y_coords)
            # print(mad)
            # base_model = lld.LinearRegressionTorch()
            # ransac_model, inlier_mask = lld.ransac_fit(x_coords, y_coords, base_model, min_samples=2, residual_threshold=5, max_trials=15)
            # a = ransac_model.coef_
            # c = ransac_model.intercept_

        # Clip the line to the frame for visualisation
        x1 = int(torch.min(x_coords).item())
        x2 = int(torch.max(x_coords).item())
        y1 = int(torch.clip(a * x1 + c, 0, self.frame_height).item())
        y2 = int(torch.clip(a * x2 + c, 0, self.frame_height).item())

        return [torch.tensor([-a.item(), 1]), (x1, y1, x2, y2)]
    
    def _calc_trigger(self, mask):
        """ Calculates whether a court line mask is in the trigger box.

        Parameters:
        - mask: numpy or tensor object containing the segmentation mask of a court line

        Returns:
        - trig_pro: the proportion of the mask in the trigger box specified as an integer representing the effective
          area of the mask in the trigger box
        - None: in the case that the court line does not enter the trigger box 
        """
        indices = torch.argwhere(mask == 1)  # coords of 1s
        line_in_box = [
            [x, y] for y, x in indices
            if self.box_x_start <= x <= self.box_x_start + self.box_width and self.box_y_start <= y <= self.box_y_start + self.box_height
        ]

        if line_in_box:
            trigger_box = mask[self.box_y_start:self.box_y_start + self.box_height, self.box_x_start:self.box_x_start + self.box_width]
            trig_pro = torch.sum(trigger_box == 1)
            return trig_pro
        else:
            return None

    def _calc_cover_frame(self, mask):
        mask_con = np.sum(mask == 0)
        mask_pro = np.sum(mask == 1)
        return "Mask coverage: " + str((mask_pro / (mask_con + mask_pro)) * 100) + "%"

    def detect(self, masks):
        """ Approximates the court lines given a YOLO segmentation mask of the court lines.
         
        Parameters:
        - mask: ultralytics.engine.results.Masks object of the segmentation masks of the court lines

        Returns:
        - (triggered, largest_line_index, all_lines, directions) where
            - triggered: is a list of integers between 0 and 2 specifying the status of each court line
                    - 0: the court line is not in the trigger box
                    - 1: the court line is in the trigger box but not the dominant line
                    - 2: the court line is in the trigger box and is the unique dominant line 
            - largest_line_index: the index of all_lines that contains the unique dominant line (or None if it doesn't exist)
            - all_lines: list that contains all of the court line equations in the form (x1, y1, x2, y2)
            - directions: list of the form [a, b] where a*x + b*y = 0, specifying the direction of the court line
        """
        all_lines = []
        directions = []
        max_prop = 0
        triggered = [0] * len(masks.data) # 0 -> not triggered, 1 -> triggered, 2 -> dominant triggered
        largest_line_index = None

        # Go through all lines
        for line_i in range(len(masks.data)):

            line = masks.data[line_i]
            line_mask = line  # 3D array of 1
            trig_amount = self._calc_trigger(line_mask)
            dir, x_lines = self._calc_single_line(line_mask, "ransac")
            all_lines.append(x_lines)
            directions.append(dir)

            if trig_amount is not None: # is the line in the trigger box?
                
                triggered[line_i] = 1

                if trig_amount > max_prop: # check if the line is dominant
                    max_prop = trig_amount
                    largest_line_index = line_i
        
        if largest_line_index is not None:
            triggered[largest_line_index] = 2

        return triggered, largest_line_index, all_lines, directions

def process_video(video_path, model, detector):
    cap = cv2.VideoCapture(video_path)

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        frame_height, frame_width, _ = frame.shape
        detections = model.predict(frame, classes=1)
        result = detections[0]
        masks = result.masks

        if masks is not None:
            triggered, line, all_lines = detector.detect(masks, orig_img=frame)
            print(triggered, line)

        cv2.imshow("Lines", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()

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
    
    # folder = "../opencv_tests/videos/"
    # image_file_names = ["2024-09-07_00-49-34-validation-converted.mp4"]

    current_image_index = 19 # 19
    image_file = folder+image_file_names[current_image_index]

    model = YOLO('best.pt')
    detections = model.predict([image_file], classes=1)  # class 1 is line
    result = detections[0]
    masks = result.masks

    visualise = False

    if visualise:
        detector = LineDetector()
        img = result.plot()  # visualise with the bounding boxes and masks
        #img = cv2.imread(image_file)  # visualise with only the image
        print(detector.detect(masks, orig_img=img))
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    else:
        detector = LineDetector()
        print(detector.detect(masks))
