import cv2
import numpy as np
import pygame
from scipy.interpolate import griddata

# Variables
CANNY_T_LOW = 50
CANNY_T_HIGH = 150
CANNY_APERTURE = 3
HOUGH_THRESHOLD = 100
HOUGH_LINE_LENGTH_MIN = 165
HOUGH_LINE_GAP_MAX = 65
CONNECT_DIST_MIN = 75
CONNECT_ANGLE_MAX = 5

image_mode_selected = False
folder = "court_lines/"
image_file_names = ['20240820_124438.jpg', '20240820_124447.jpg','20240820_124451.jpg','20240820_124509.jpg','20240820_124511.jpg','20240820_124521.jpg','20240820_124524.jpg','20240820_124535.jpg','20240820_124539.jpg','20240820_124551.jpg']
current_image_index = 0

WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
LIGHT_GREY = (200, 200, 200)
BLUE = (0, 0, 255)

class Slider:
    def __init__(self, x, y, w, min_val, max_val, start_val, step=1):
        self.rect = pygame.Rect(x, y, w, 20)
        self.min_val = min_val
        self.max_val = max_val
        self.value = start_val
        self.step = step
        self.circle_pos = self.get_circle_pos()
        self.input_active = False
        self.input_box = pygame.Rect(self.rect.right + 10, self.rect.y, 50, 20)
        self.input_text = str(int(self.value))

    def get_circle_pos(self):
        ratio = (self.value - self.min_val) / (self.max_val - self.min_val)
        return (self.rect.x + int(ratio * self.rect.width), self.rect.y + 10)

    def draw(self, screen):
        pygame.draw.rect(screen, BLACK, self.rect, 2)
        pygame.draw.circle(screen, BLACK, self.circle_pos, 10)
        
        # Draw input box
        pygame.draw.rect(screen, LIGHT_GREY if self.input_active else WHITE, self.input_box)
        pygame.draw.rect(screen, BLACK, self.input_box, 2)
        
        font = pygame.font.Font(None, 24)
        text_surface = font.render(self.input_text, True, BLACK)
        screen.blit(text_surface, (self.input_box.x + 5, self.input_box.y))

    def update(self, pos):
        if self.rect.collidepoint(pos):
            ratio = (pos[0] - self.rect.x) / self.rect.width
            self.value = self.min_val + round(ratio * (self.max_val - self.min_val) / self.step) * self.step
            self.value = max(min(self.value, self.max_val), self.min_val)
            self.circle_pos = self.get_circle_pos()
            self.input_text = str(int(self.value))

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN:
            if self.input_box.collidepoint(event.pos):
                self.input_active = True
            else:
                self.input_active = False

        if event.type == pygame.KEYDOWN and self.input_active:
            if event.key == pygame.K_RETURN:
                # When Enter is pressed, update the slider value
                try:
                    entered_value = int(self.input_text)
                    self.value = max(min(entered_value, self.max_val), self.min_val)
                    self.circle_pos = self.get_circle_pos()
                except ValueError:
                    pass
                self.input_active = False
            elif event.key == pygame.K_BACKSPACE:
                self.input_text = self.input_text[:-1]
            else:
                if event.unicode.isdigit() or (event.unicode == '-' and len(self.input_text) == 0):
                    self.input_text += event.unicode

class Button:
    def __init__(self, x, y, w, h, text):
        self.rect = pygame.Rect(x, y, w, h)
        self.text = text
        self.active = False

    def draw(self, screen):
        color = BLUE if self.active else BLACK
        pygame.draw.rect(screen, color, self.rect, 2)
        font = pygame.font.Font(None, 30)
        text_surface = font.render(self.text, True, color)
        text_rect = text_surface.get_rect(center=self.rect.center)
        screen.blit(text_surface, text_rect)

    def is_clicked(self, pos):
        return self.rect.collidepoint(pos)

class ArrowButton:
    def __init__(self, x, y, direction):
        self.rect = pygame.Rect(x, y, 40, 40)
        self.direction = direction

    def draw(self, screen):
        pygame.draw.rect(screen, BLACK, self.rect, 2)
        font = pygame.font.Font(None, 36)
        arrow_text = "<" if self.direction == "left" else ">"
        text_surface = font.render(arrow_text, True, BLACK)
        text_rect = text_surface.get_rect(center=self.rect.center)
        screen.blit(text_surface, text_rect)

    def is_clicked(self, pos):
        return self.rect.collidepoint(pos)
    
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

if __name__ == "__main__":
    #image_path = 'stock_image_cropped.jpg'
    #balls = detect_balls('stock_image_cropped.jpg')
    #lines = detect_line('tennis_court.jpg')
    #close_lines = detect_cline('tennis_court.jpg')

    # Settings adjustment GUI
    pygame.init()
    width, height = 700, 650
    screen = pygame.display.set_mode((width, height))
    pygame.display.set_caption('Adjust CV Settings')

    sliders = {
        "CANNY_T_LOW": Slider(300, 60, 250, 0, 255, CANNY_T_LOW),
        "CANNY_T_HIGH": Slider(300, 100, 250, 0, 255, CANNY_T_HIGH),
        "CANNY_APERTURE": Slider(300, 140, 250, 3, 7, CANNY_APERTURE, step=2),
        "HOUGH_THRESHOLD": Slider(300, 220, 250, 0, 255, HOUGH_THRESHOLD),
        "HOUGH_LINE_LENGTH_MIN": Slider(300, 260, 250, 0, 255, HOUGH_LINE_LENGTH_MIN),
        "HOUGH_LINE_GAP_MAX": Slider(300, 300, 250, 0, 255, HOUGH_LINE_GAP_MAX),
        "CONNECT_DIST_MIN": Slider(300, 380, 250, 0, 255, CONNECT_DIST_MIN),
        "CONNECT_ANGLE_MAX": Slider(300, 420, 250, 0, 30, CONNECT_ANGLE_MAX)
    }

    camera_button = Button(100, 500, 100, 40, "Camera")
    image_button = Button(220, 500, 100, 40, "Image")
    left_arrow = ArrowButton(100, 550, "left")
    right_arrow = ArrowButton(350, 550, "right")

    cap = cv2.VideoCapture(1)
    running = True

    while running:
        screen.fill(WHITE)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                pos = event.pos
                if pygame.mouse.get_pressed()[0]:  # Check if the left mouse button is held down
                    for name, slider in sliders.items():
                        slider.update(pos)
                        globals()[name] = slider.value  # Update the corresponding variable

                    if camera_button.is_clicked(pos):
                        camera_button.active = True
                        image_button.active = False
                        image_mode_selected = False

                    if image_button.is_clicked(pos):
                        image_button.active = True
                        camera_button.active = False
                        image_mode_selected = True

                    if image_mode_selected:
                        if left_arrow.is_clicked(pos):
                            current_image_index = (current_image_index - 1) % len(image_file_names)
                        if right_arrow.is_clicked(pos):
                            current_image_index = (current_image_index + 1) % len(image_file_names)
            
            elif event.type == pygame.KEYDOWN or event.type == pygame.MOUSEBUTTONDOWN:
                for slider in sliders.values():
                    slider.handle_event(event)

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

        draw_text(screen, "CV Analysis", (50, 470), color=RED, size=36)
        camera_button.draw(screen)
        image_button.draw(screen)

        # Show arrows and image filename if image mode is selected
        if image_mode_selected:
            left_arrow.draw(screen)
            right_arrow.draw(screen)

            frame_orig = cv2.imread(folder+image_file_names[current_image_index])
            frame = cv2.resize(frame_orig, (1008, 756), interpolation=cv2.INTER_AREA)
            draw_text(screen, image_file_names[current_image_index], (150, 560), size=25)
        else:
            # camera
            ret, frame = cap.read()

            if not ret:
                break

        # Update the display
        pygame.display.flip()

        #detect_balls(frame)
        detect_line(frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    pygame.quit()
