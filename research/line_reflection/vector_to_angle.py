__author__ = "Alex Jones"
__date__ = "12/09/2024"


import math
import numpy as np

def vector_to_angle(vector):
    # Converts target direction into number of degrees to the right to turn
    # Input vector is normalised target direction as numpy array
    turn_fraction = vector[1] / vector[0]
    arctan_value = 0
    turn_angle = 0
    if vector[0] < 0:
        arctan_value = math.atan(-1 * turn_fraction)
        turn_angle = 180 - (arctan_value * 180 / math.pi)

    else:
        arctan_value = math.atan(turn_fraction)
        turn_angle = arctan_value * 180 / math.pi

    # Turn angle is number of degrees to the right, negative values for left turn
    return turn_angle
    


    

if __name__ == "__main__":
    test_vector = np.array([1,1])
    result = vector_to_angle(test_vector)
    print(result)