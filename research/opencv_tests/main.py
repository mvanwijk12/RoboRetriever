import cv2
import numpy as np
import pygame
from scipy.interpolate import griddata

# Variables
CANNY_T_LOW = 50
CANNY_T_HIGH = 150
CANNY_APERTURE = 3
HOUGH_THRESHOLD = 125
HOUGH_LINE_LENGTH_MIN = 165
HOUGH_LINE_GAP_MAX = 65
CONNECT_DIST_MIN = 75
CONNECT_ANGLE_MAX = 4
TRIGGER_SIZE = 0.5
TRIGGER_OFFSET = 0.2

image_mode_selected = True
folder = "court_lines/"
image_file_names = ['20240820_124734.jpg','20240820_124438.jpg','20240820_124447.jpg','20240820_124451.jpg','20240820_124509.jpg','20240820_124511.jpg',
                    '20240820_124521.jpg','20240820_124524.jpg','20240820_124535.jpg','20240820_124539.jpg','20240820_124551.jpg','20240820_124554.jpg',
                    '20240820_124603.jpg','20240820_124612.jpg','20240820_124619.jpg','20240820_124626.jpg','20240820_124701.jpg','20240820_124708.jpg',
                    '20240820_124732.jpg','20240820_124745.jpg','20240820_124802.jpg','20240820_124804.jpg','20240820_124806.jpg','20240820_124808.jpg',
                    '20240820_124810.jpg','20240820_124820.jpg','20240820_124821.jpg','20240820_124826.jpg','20240820_124827.jpg','20240820_124831.jpg',
                    '20240820_124833.jpg','20240820_124835.jpg','20240820_124838.jpg']
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

def detect_line(frame, show_img=False):
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
        for line in connected_lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(line_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

    # Define the trigger box (center of the image with a given size)
    height, width = frame.shape[:2]
    height_offset = int(height * TRIGGER_OFFSET)
    box_center = (width // 2, height // 2 + height_offset)
    box_size = int(min(width, height) * TRIGGER_SIZE)
    box_x1 = box_center[0] - box_size // 2
    box_y1 = box_center[1] - box_size // 2
    box_x2 = box_center[0] + box_size // 2
    box_y2 = box_center[1] + box_size // 2

    # Draw the trigger box on the image
    cv2.rectangle(line_image, (box_x1, box_y1), (box_x2, box_y2), RED, 2)

    # Check if any line crosses the trigger box
    triggered = False
    angle = None
    distance = None
    line_vector = [0,0]
    if connected_lines is not None:
        for line in connected_lines:
            x1, y1, x2, y2 = line[0]

            # Check for intersection with the trigger box boundaries
            intersections = []
            if box_x1 <= x1 <= box_x2 and box_y1 <= y1 <= box_y2:
                intersections.append((x1, y1))
            if box_x1 <= x2 <= box_x2 and box_y1 <= y2 <= box_y2:
                intersections.append((x2, y2))
            
            # Calculate intersection with the box edges if endpoints are outside
            def line_intersection(p1, p2, q1, q2):
                """ Find the intersection of two lines (p1-p2) and (q1-q2). """
                s1_x, s1_y = p2[0] - p1[0], p2[1] - p1[1]
                s2_x, s2_y = q2[0] - q1[0], q2[1] - q1[1]

                s = (-s1_y * (p1[0] - q1[0]) + s1_x * (p1[1] - q1[1])) / (-s2_x * s1_y + s1_x * s2_y)
                t = ( s2_x * (p1[1] - q1[1]) - s2_y * (p1[0] - q1[0])) / (-s2_x * s1_y + s1_x * s2_y)

                if 0 <= s <= 1 and 0 <= t <= 1:
                    int_x = p1[0] + (t * s1_x)
                    int_y = p1[1] + (t * s1_y)
                    return int(int_x), int(int_y)
                return None

            # Check intersection with each box edge
            box_edges = [((box_x1, box_y1), (box_x2, box_y1)),
                         ((box_x2, box_y1), (box_x2, box_y2)),
                         ((box_x2, box_y2), (box_x1, box_y2)),
                         ((box_x1, box_y2), (box_x1, box_y1))]

            for edge in box_edges:
                intersection = line_intersection((x1, y1), (x2, y2), edge[0], edge[1])
                if intersection is not None:
                    intersections.append(intersection)

            # If we have two intersections, the line crosses the box
            if len(intersections) == 2:
                triggered = True
                midpoint_x = (intersections[0][0] + intersections[1][0]) // 2
                midpoint_y = (intersections[0][1] + intersections[1][1]) // 2
                
                # Calculate the angle from the box center to the midpoint
                angle = np.arctan2(midpoint_y - box_center[1], midpoint_x - box_center[0]) * 180 / np.pi
                angle = (angle + 450) % 360 # convert to 0 to 360 instead of -180 to 180

                # Calculate the distance form the box center to the midpoint
                distance = np.sqrt((midpoint_x - box_center[0])**2 + (midpoint_y - box_center[1])**2)

                # Calculate the vector (ax+by=c into [a,b]) of the line
                vec_a = y2 - y1
                vec_b = x1 - x2
                vec_c = vec_a * x1 + vec_b * y1
                line_vector = [vec_a,vec_b]

                break  # Exit loop once a crossing line is found

    # display image
    combined_image = cv2.addWeighted(frame, 0.5, line_image, 1, 0)
    if show_img:
        cv2.imshow('Detected Lines', combined_image)

    return connected_lines, [triggered, angle, distance, line_vector], combined_image

if __name__ == "__main__":
    # Settings adjustment GUI
    pygame.init()
    width, height = 650, 730
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
        "CONNECT_ANGLE_MAX": Slider(300, 420, 250, 0, 30, CONNECT_ANGLE_MAX),
        "TRIGGER_SIZE": Slider(300, 500, 250, 0.1, 0.9, TRIGGER_SIZE, step=0.1),
        "TRIGGER_OFFSET": Slider(300, 540, 250, 0, 0.5, TRIGGER_OFFSET, step=0.1)
    }

    slider_values_changed = False

    camera_button = Button(50, 620, 100, 40, "Camera")
    image_button = Button(170, 620, 100, 40, "Image")
    left_arrow = ArrowButton(50, 670, "left")
    right_arrow = ArrowButton(300, 670, "right")

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
                        slider_values_changed = True

                    if camera_button.is_clicked(pos):
                        camera_button.active = True
                        image_button.active = False
                        image_mode_selected = False
                        slider_values_changed = False

                    if image_button.is_clicked(pos):
                        image_button.active = True
                        camera_button.active = False
                        image_mode_selected = True
                        slider_values_changed = True

                    if image_mode_selected:
                        if left_arrow.is_clicked(pos):
                            current_image_index = (current_image_index - 1) % len(image_file_names)
                        if right_arrow.is_clicked(pos):
                            current_image_index = (current_image_index + 1) % len(image_file_names)
            
            elif event.type == pygame.KEYDOWN or event.type == pygame.TEXTINPUT:
                for slider in sliders.values():
                    if slider.input_active:
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

        draw_text(screen, "Trigger Box", (50, 460), color=RED, size=36)
        draw_text(screen, "TRIGGER_SIZE", (50, 500))
        sliders["TRIGGER_SIZE"].draw(screen)
        draw_text(screen, "TRIGGER_OFFSET", (50, 540))
        sliders["TRIGGER_OFFSET"].draw(screen)

        draw_text(screen, "CV Analysis", (50, 580), color=RED, size=36)
        camera_button.draw(screen)
        image_button.draw(screen)

        # Show arrows and image filename if image mode is selected
        if image_mode_selected:
            left_arrow.draw(screen)
            right_arrow.draw(screen)

            frame_orig = cv2.imread(folder+image_file_names[current_image_index])
            frame = cv2.resize(frame_orig, (504, 378), interpolation=cv2.INTER_AREA)
            draw_text(screen, image_file_names[current_image_index], (100, 680), size=25)

            if slider_values_changed:
                lines_list, trigger, image = detect_line(frame)
                cv2.imshow('Detected Lines', image)
                slider_values_changed = False

                print("\n#####################")

                # Provide output if the trigger box is crossed
                if trigger[0]:
                    print(f"Trigger Box Crossed! Angle: {trigger[1]:.2f} degrees, Distance: {trigger[2]:.2f} px")
                    print(trigger[3])
                else:
                    print("No crossing detected.")

                #print("   x1  y1  x2  y2")
                #print(lines_list)

        else:
            ret, frame = cap.read()
            if not ret:
                break
            lines_list, trigger, image = detect_line(frame)
            cv2.imshow('Detected Lines', image)

            print("\n#####################")

            # Provide output if the trigger box is crossed
            if trigger[0]:
                print(f"Trigger Box Crossed! Angle: {trigger[1]:.2f} degrees, Distance: {trigger[2]:.2f} px")
                print(trigger[3])
            else:
                print("No crossing detected.")
        
            #print("   x1  y1  x2  y2")
            #print(lines_list)
            
        # Update the display
        pygame.display.flip()

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    pygame.quit()
