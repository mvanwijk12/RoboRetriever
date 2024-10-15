import numpy as np

def sutherland_hodgman(polygon, clip_window):
    def inside(p, edge_start, edge_end):
        # Check if point p is inside the edge (using the cross product)
        return (edge_end[0] - edge_start[0]) * (p[1] - edge_start[1]) - (edge_end[1] - edge_start[1]) * (p[0] - edge_start[0]) >= 0

    def intersection(p1, p2, edge_start, edge_end):
        # Find intersection point between line (p1, p2) and the clipping edge (edge_start, edge_end)
        x1, y1 = p1
        x2, y2 = p2
        x3, y3 = edge_start
        x4, y4 = edge_end
        
        denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
        if denom == 0:  # Lines are parallel
            return None
        
        t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom
        intersect_x = x1 + t * (x2 - x1)
        intersect_y = y1 + t * (y2 - y1)
        
        return np.array([intersect_x, intersect_y])

    output_list = polygon

    # Go through each edge of the clip window
    for i in range(len(clip_window)):
        edge_start = clip_window[i]
        edge_end = clip_window[(i + 1) % len(clip_window)]

        input_list = output_list
        output_list = []

        if len(input_list) == 0:
            break

        s = input_list[-1]  # Start from the last point

        for p in input_list:
            if inside(p, edge_start, edge_end):  # Case 1: Current point is inside
                if not inside(s, edge_start, edge_end):  # Case 1.1: Previous point is outside
                    output_list.append(intersection(s, p, edge_start, edge_end))  # Add intersection
                output_list.append(p)  # Add current point
            elif inside(s, edge_start, edge_end):  # Case 2: Current point is outside, previous point is inside
                output_list.append(intersection(s, p, edge_start, edge_end))  # Add intersection
            s = p  # Update start point to the current point

    return np.array(output_list)

def polygon_area(polygon):
    x = polygon[:, 0]
    y = polygon[:, 1]
    return 0.5 * np.abs(np.dot(x, np.roll(y, 1)) - np.dot(y, np.roll(x, 1)))

# Example usage
polygon = np.array([[2, 1], [4, 5], [7, 8], [1, 7]])  # Polygon vertices
rectangle = np.array([[3, 2], [6, 2], [6, 6], [3, 6]])  # Rectangle [min_x, min_y, max_x, max_y]

# Clip the polygon with the rectangle
clipped_polygon = sutherland_hodgman(polygon, rectangle)

if len(clipped_polygon) > 0:
    intersection_area = polygon_area(clipped_polygon)
    print(f"The intersection area is: {intersection_area}")
else:
    print("No intersection between the polygon and the rectangle")
