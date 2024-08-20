import cv2
import numpy as np
import pygame
from scipy.interpolate import griddata

# Variables
CANNY_T_LOW = 50
CANNY_T_HIGH = 150
CANNY_APERTURE = 5
HOUGH_THRESHOLD = 70
HOUGH_LINE_LENGTH_MIN = 60
HOUGH_LINE_GAP_MAX = 20
CONNECT_DIST_MIN = 30
CONNECT_ANGLE_MAX = 10

WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)

class Slider:
    def __init__(self, x, y, w, min_val, max_val, start_val, step=1):
        self.rect = pygame.Rect(x, y, w, 20)
        self.min_val = min_val
        self.max_val = max_val
        self.value = start_val
        self.step = step
        self.circle_pos = self.get_circle_pos()

    def get_circle_pos(self):
        ratio = (self.value - self.min_val) / (self.max_val - self.min_val)
        return (self.rect.x + int(ratio * self.rect.width), self.rect.y + 10)

    def draw(self, screen):
        pygame.draw.rect(screen, BLACK, self.rect, 2)
        pygame.draw.circle(screen, BLACK, self.circle_pos, 10)
        font = pygame.font.Font(None, 24)
        value_text = font.render(f'{int(self.value)}', True, BLACK)
        screen.blit(value_text, (self.rect.right + 10, self.rect.y))

    def update(self, pos):
        if self.rect.collidepoint(pos):
            ratio = (pos[0] - self.rect.x) / self.rect.width
            self.value = self.min_val + round(ratio * (self.max_val - self.min_val) / self.step) * self.step
            self.value = max(min(self.value, self.max_val), self.min_val)
            self.circle_pos = self.get_circle_pos()

def draw_text(screen, text, pos, color=BLACK, size=30):
    font = pygame.font.Font(None, size)
    text_surface = font.render(text, True, color)
    screen.blit(text_surface, pos)


def translate_points(circles):
    calibration_data = {
        (100, 100): 0.05,  # at (100, 100) pixels, 1 pixel = 0.05 meters
        (200, 200): 0.04,  # at (200, 200) pixels, 1 pixel = 0.04 meters
        # Add more calibrated points as necessary
    }

    # prepare data for interpolation
    points = np.array(list(calibration_data.keys()))
    values = np.array(list(calibration_data.values()))

    real_world_circles = []
    for circle in circles:
        center = circle["center"]
        radius = circle["radius"]
        scaling_factor = griddata(points, values, center, method='linear')
        real_world_radius = radius * scaling_factor
        real_world_circles.append({"center": center, "radius": real_world_radius})

    # print the results
    for circle in real_world_circles:
        print(f"Center: {circle['center']}, Real-world radius: {circle['radius']} meters")

def detect_balls(frame):
    # convert to HSV colour space
    hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # set HSV range for tennis ball colours - can be fine tuned
    lower_hsv = np.array([29, 86, 6])
    upper_hsv = np.array([64, 255, 255])

    # create mask for the HSV colours and add blur to reduce noise
    mask = cv2.inRange(hsv_image, lower_hsv, upper_hsv)
    blurred_mask = cv2.GaussianBlur(mask, (9, 9), 2)  # (9,9) 

    # remove more noise and improve circular shape
    kernel = np.ones((5,5), np.uint8)
    opened_mask = cv2.morphologyEx(blurred_mask, cv2.MORPH_OPEN, kernel)
    closed_mask = cv2.morphologyEx(opened_mask, cv2.MORPH_CLOSE, kernel)

    # find contours
    contours, _ = cv2.findContours(closed_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # TODO: calibration to provide real world values

    for contour in contours:
        (x,y), radius = cv2.minEnclosingCircle(contour)
        centre = (int(x), int(y))
        radius = int(radius)  # this is our distance measurement
        print("Centre: ", centre, " & radius: ", radius)

        if radius > 10:
            cv2.circle(frame, centre, radius, (0,255,0), 2)
            cv2.circle(frame, centre, 2, (0,0,255), 3)
            
    # display image
    #cv2.imshow('Mask', closed_mask)
    cv2.imshow('Detected Tennis Balls', frame)

def detect_line(frame):
    # convert to greyscale
    grey_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # apply gaussian blur
    blurred = cv2.GaussianBlur(grey_image, (5, 5), 0)

    # adaptive thresholding for different shades of white of lines
    adaptive_thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
    
    # morphological operations to close gaps in lines
    kernel = np.ones((3, 3), np.uint8)
    dilated = cv2.dilate(adaptive_thresh, kernel, iterations=1)
    eroded = cv2.erode(dilated, kernel, iterations=1)

    # edge detection
    edges = cv2.Canny(eroded, CANNY_T_LOW, CANNY_T_HIGH, apertureSize=CANNY_APERTURE)

    # Hough line transform
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=HOUGH_THRESHOLD, minLineLength=HOUGH_LINE_LENGTH_MIN, maxLineGap=HOUGH_LINE_GAP_MAX)

    # draw lines
    line_image = np.zeros_like(frame)

    # Function to connect line segments that are close and aligned
    def connect_lines(lines, min_distance=CONNECT_DIST_MIN, angle_threshold=CONNECT_ANGLE_MAX):
        if lines is None:
            return []
        
        new_lines = []
        for i, line1 in enumerate(lines):
            for j, line2 in enumerate(lines):
                if i != j:
                    x1, y1, x2, y2 = line1[0]
                    x3, y3, x4, y4 = line2[0]
                    dist = np.linalg.norm([x2 - x3, y2 - y3])
                    angle_diff = abs(np.arctan2(y2 - y1, x2 - x1) - np.arctan2(y4 - y3, x4 - x3)) * 180 / np.pi
                    if dist < min_distance and angle_diff < angle_threshold:
                        new_lines.append([[x1, y1, x4, y4]])
                    else:
                        new_lines.append(line1)
                        new_lines.append(line2)
        
        return np.array(new_lines)
    
    # Connect broken line segments
    connected_lines = connect_lines(lines)
    
    if connected_lines is not None:
        #print("   x1  y1  x2  y2")
        for line in connected_lines:
            x1, y1, x2, y2 = line[0]
            #print(line)
            cv2.line(line_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

    combined_image = cv2.addWeighted(frame, 0.5, line_image, 1, 0)

    # display image
    #cv2.imshow('Adaptive Threshold', adaptive_thresh)
    #cv2.imshow('Eroded', eroded)
    #cv2.imshow('Canny', edges)
    cv2.imshow('Detected Lines', combined_image)

def detect_cline(frame):
    # convert to greyscale
    grey_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # apply gaussian blur
    blurred = cv2.GaussianBlur(grey_image, (5, 5), 0)

    # adaptive thresholding for different shades of white of lines
    adaptive_thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
    
    # morphological operations to close gaps in lines
    kernel = np.ones((3, 3), np.uint8)
    dilated = cv2.dilate(adaptive_thresh, kernel, iterations=1)
    eroded = cv2.erode(dilated, kernel, iterations=1)

    # edge detection
    edges = cv2.Canny(eroded, 50, 150)

    # Hough line transform
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=80, minLineLength=30, maxLineGap=20)
    
    # get image center
    center_x, center_y = frame.shape[1] // 2, frame.shape[0] // 2

    # calculate distance from center
    def distance_from_center(x1, y1, x2, y2, cx, cy):
        mid_x, mid_y = (x1 + x2) / 2, (y1 + y2) / 2
        return np.sqrt((mid_x - cx) ** 2 + (mid_y - cy) ** 2)
    
    # filter lines based on distance from center
    max_distance = 110
    filtered_lines = []
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            distance = distance_from_center(x1, y1, x2, y2, center_x, center_y)
            if distance < max_distance:
                filtered_lines.append(line)
    
    # draw lines on
    line_image = np.zeros_like(frame)
    for line in filtered_lines:
        x1, y1, x2, y2 = line[0]
        cv2.line(line_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
    
    combined_image = cv2.addWeighted(frame, 0.8, line_image, 1, 0)
    
    # display image
    #cv2.imshow('Adaptive Threshold', adaptive_thresh)
    #cv2.imshow('Eroded', eroded)
    #cv2.imshow('Canny', edges)
    cv2.imshow('Detected Lines', combined_image)

if __name__ == "__main__":
    #image_path = 'stock_image_cropped.jpg'
    #balls = detect_balls('stock_image_cropped.jpg')
    #lines = detect_line('tennis_court.jpg')
    #close_lines = detect_cline('tennis_court.jpg')

    # Settings adjustment GUI
    pygame.init()
    width, height = 600, 500
    screen = pygame.display.set_mode((width, height))
    pygame.display.set_caption('Adjust Settings')

    sliders = {
        "CANNY_T_LOW": Slider(300, 60, 250, 0, 255, 50),
        "CANNY_T_HIGH": Slider(300, 100, 250, 0, 255, 150),
        "CANNY_APERTURE": Slider(300, 140, 250, 3, 7, 3, step=2),
        "HOUGH_THRESHOLD": Slider(300, 220, 250, 0, 255, 70),
        "HOUGH_LINE_LENGTH_MIN": Slider(300, 260, 250, 0, 255, 60),
        "HOUGH_LINE_GAP_MAX": Slider(300, 300, 250, 0, 255, 20),
        "CONNECT_DIST_MIN": Slider(300, 380, 250, 0, 255, 30),
        "CONNECT_ANGLE_MAX": Slider(300, 420, 250, 0, 30, 10)
    }

    cap = cv2.VideoCapture(1)

    while True:
        ret, frame = cap.read()
        screen.fill(WHITE)

        if not ret:
            break

        # Event handling
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEBUTTONDOWN or event.type == pygame.MOUSEMOTION:
                if pygame.mouse.get_pressed()[0]:  # Check if the left mouse button is held down
                    for slider in sliders.values():
                        slider.update(event.pos)

        # Draw headings and sliders
        draw_text(screen, "Canny Edge Detection", (50, 20), color=RED, size=36)
        draw_text(screen, "CANNY_T_LOW", (50, 60))
        sliders["CANNY_T_LOW"].draw(screen)
        draw_text(screen, "CANNY_T_HIGH", (50, 100))
        sliders["CANNY_T_HIGH"].draw(screen)
        draw_text(screen, "CANNY_APERTURE", (50, 140))
        sliders["CANNY_APERTURE"].draw(screen)

        draw_text(screen, "Hough Line Transform", (50, 180), color=RED, size=36)
        draw_text(screen, "HOUGH_THRESHOLD", (50, 220))
        sliders["HOUGH_THRESHOLD"].draw(screen)
        draw_text(screen, "HOUGH_LINE_LENGTH_MIN", (50, 260))
        sliders["HOUGH_LINE_LENGTH_MIN"].draw(screen)
        draw_text(screen, "HOUGH_LINE_GAP_MAX", (50, 300))
        sliders["HOUGH_LINE_GAP_MAX"].draw(screen)

        draw_text(screen, "Line Connection", (50, 340), color=RED, size=36)
        draw_text(screen, "CONNECT_DIST_MIN", (50, 380))
        sliders["CONNECT_DIST_MIN"].draw(screen)
        draw_text(screen, "CONNECT_ANGLE_MAX", (50, 420))
        sliders["CONNECT_ANGLE_MAX"].draw(screen)

        # Update the display
        pygame.display.flip()

        #detect_balls(frame)
        #detect_line(frame)
        #detect_cline(frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    pygame.quit()
