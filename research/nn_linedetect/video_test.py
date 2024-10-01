from ultralytics import YOLO
import numpy as np
import cv2
import logging
import logging.config

class LineDetector:
    def __init__(self, box_width_ratio=0.5, box_height_ratio=0.15, bottom_offset_ratio=0.03, viz_type=1):
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
    cv2.destroyAllWindows()

if __name__ == "__main__":
    logger = logging.getLogger(__name__)

    video_path = "../opencv_tests/videos/2024-09-07_00-49-34-validation-converted.mp4"
    model = YOLO('best.pt')
    detector = LineDetector(viz_type=1)
    
    process_video(video_path, model, detector)
