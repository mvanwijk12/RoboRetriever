# This is a test loop for the robot to drive

import os
os.system("sudo pigpiod")
import math
import pigpio
import sys
import time
import RPi.GPIO as GPIO
import logging
import logging.config
from drive_B_with_radius import Drive_B

if __name__ == "__main__":
    logging.config.fileConfig('log.conf')
    logger = logging.getLogger(__name__)
    robot = Drive_B()

    lap_length = float(input("Enter loop length (first side) in m: "))
    lap_width = float(input("Enter loop width (second side) in m: "))
    turn_radius = float(input("Enter radius of turns in m: "))
    speed = float(input("Enter speed in m/s: "))
    turn_rate = float(input("Enter turn rate in degrees/s: "))
    orientation = -1
    while orientation != 0 and orientation != 1:
        orientation = int(input("Enter 0 for clockwise loop or 1 for anti-clockwise loop: "))
    
    # Convert orientation to multiplier for angles
    if orientation == 1:
        orientation = -1
    else:
        orientation = 1

    while True:
        try:
            print(f'Test lap is {lap_length} m long and {lap_width} m wide')
            input("Press ENTER to start test lap")

            # First leg is longer as robot starts in corner
            robot.straight_drive(lap_length - turn_radius, speed)
            robot.turn_to_angle(90 * orientation, turn_rate, turn_radius)
            robot.straight_drive(lap_width - 2 * turn_radius, speed)
            robot.turn_to_angle(90 * orientation, turn_rate, turn_radius)
            robot.straight_drive(lap_length - 2 * turn_radius, speed)
            robot.turn_to_angle(90 * orientation, turn_rate, turn_radius)
            # Last leg is longer and finishes with turn on the spot at the original location
            robot.straight_drive(lap_width - turn_radius, speed)
            robot.turn_to_angle(90 * orientation, turn_rate, 0)
        
            print("Lap complete")

        except Exception as e:
            # terminate gracefully
            robot.logger.info(f'Exception: {e}')
            robot.all_stop()
            GPIO.cleanup()
            sys.exit()

        except KeyboardInterrupt:
            # terminate gracefully
            robot.all_stop()
            GPIO.cleanup()
            sys.exit()