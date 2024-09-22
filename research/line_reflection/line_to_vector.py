__author__ = "Alex Jones"
__date__ = "04/09/2024"


import math
import numpy as np

def line_to_vector(line):
    # Input is line in the form ax+by=c, only uses a and b
    initial_direction = np.array([0,1])
    normal_direction = line/np.linalg.norm(line)
    reflected_direction = initial_direction - (2*np.dot(initial_direction,normal_direction))*normal_direction
    return reflected_direction/np.linalg.norm(reflected_direction)

if __name__ == "__main__":
    test_line = np.array([2,3])
    result = line_to_vector(test_line)
    print(result)