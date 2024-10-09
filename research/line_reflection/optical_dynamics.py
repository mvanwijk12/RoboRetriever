__author__ = "Alex Jones"
__edited__ = "Matt van Wijk"
__date__ = "28/09/2024"


import math
import numpy as np

def reflect_line_to_angle(rline):
    """ Converts the reflected line in the form a*x + b*y = 0 to a turn angle for the robot to move assuming optical dynamics. 
    
    :param rline: reflected line equation as a numpy array [a, b] where a*x + b*y = 0
    :returns: angle in degrees for the robot to turn in the rightward direction, negative angle corresponds to a leftward turn 
    """
    return math.atan2(rline[1], -rline[0]) * 180/math.pi
    
def reflection_line(line):
    """ Calculates the equation of the reflected line for an incidence direction of (x, y) = (0, 1) and a mirror line given by line.
     The reflected line is calculated assuming optical dynamics.
     
    :param line: mirror line equation as a numpy array [a, b] where a*x + b*y = 0
    :returns: reflected line equation as a numpy array [c, d] where c*x + d*y = 0
    """
    incidence_direction = np.array([1,0])
    normalized_mirror_direction = line/np.linalg.norm(line)
    reflected_direction = incidence_direction - (2*np.dot(incidence_direction, normalized_mirror_direction))*normalized_mirror_direction
    return reflected_direction/np.linalg.norm(reflected_direction)
    

if __name__ == "__main__":
    test_lines = [[0, 1], [1, 1], [-1, 1]]
    for line in test_lines:
        print(f'line is {line[0]}x + {line[1]}y = 0')
        test_line = np.array(line)
        rline = reflection_line(test_line)
        print(f'reflected line is {rline[0]}x + {rline[1]}y = 0')
        angle = reflect_line_to_angle(rline)
        print(f'turn angle is {angle}deg in rightward direction\n')