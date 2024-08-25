__author__ = "Alex Jones"
__date__ = "25/08/2024"

# Approximations fall apart if ball is too close to the camera
# If the ball is far away, the edge of the ball that the camera sees has tangent approx. parallel to the line to the centre of the ball

import math

def distance_converter(x1, x2):
    horizontal_res = 1280
    fov = 102 # From camera datasheet
    diameter = 66.75 # Middle of standard range

    x_diff = abs(x2 - x1)
    angle_diff = x_diff * fov / horizontal_res # Represents angle from centre of ball to edge
    distance = diameter / math.tan(math.pi / 180 * angle_diff) # Should use ball radius and 1/2 of angle, but this works
    return distance