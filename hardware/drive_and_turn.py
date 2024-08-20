__author__ = "Matt van Wijk"
__date__ = "13/08/2024"

# Modified by Gael to add turn

import pigpio
import math
import os
import sys
from time import sleep
import time
import threading
from datetime import datetime, timedelta
import RPi.GPIO as GPIO

class Drive:
    """ Represents a robot drive object """
    def __init__(self, stepL=18, stepR=19, dirL=24, dirR=23):
        """ Initalise the Drive Object
        
        :param stepR: PWM pin connected to the right stepper motor driver step pin
        :param stepL: PWM pin connected to the left stepper motor driver step pin
        :param dirR: GPIO pin connected to the right stepper motor driver direction pin
        :param dirL: GPIO pin connected to the left stepper motor driver direction pin
        """
        self.stepL = stepL
        self.stepR = stepR
        self.dirL = dirL
        self.dirR = dirR
        self.wheel_circumference = 55e-3 * math.pi # measured wheel diameter 55mm
        self.traction_factor = 1
        self.steps_per_rev = 200
        self.stepping_mode = 1/8 # Assume 1/8 stepping
        self.pi = pigpio.pi()

        os.system("sudo pigpiod") # start the pigpiod daemon
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.dirL, GPIO.OUT)
        GPIO.setup(self.dirR, GPIO.OUT)

    def set_1D_direction(self, dirForward=True):
        """ Set the drive direction as either forwards or backwards """
        if dirForward:
            GPIO.output(self.dirL, GPIO.HIGH)
            GPIO.output(self.dirR, GPIO.LOW)
        else:
            GPIO.output(self.dirL, GPIO.LOW)
            GPIO.output(self.dirR, GPIO.HIGH)

    def drive(self, distance=1, speed=1):
        """ Function to start driving a specified distance in metres """
        req_revolutions = distance/(self.traction_factor * self.wheel_circumference)
        req_steps = req_revolutions * self.steps_per_rev * (1/self.stepping_mode)
        req_revs_per_sec = speed/(self.traction_factor * self.wheel_circumference)
        req_steps_per_sec = req_revs_per_sec * self.steps_per_rev * (1/self.stepping_mode) # PWM freq
        drive_time = req_steps/(req_steps_per_sec)

        self.setup_timer("Timer 1", drive_time, self.timer_function) 
        self.pi.hardware_PWM(self.stepL, int(req_steps_per_sec), 500000)
        self.pi.hardware_PWM(self.stepR, int(req_steps_per_sec), 500000)

    def turn_left(self, speed=1):
        """ Function to make the robot turn left """
        # Reverse the right motor and keep the left motor forward
        GPIO.output(self.dirL, GPIO.HIGH)
        GPIO.output(self.dirR, GPIO.HIGH)  # Reverse right motor

        req_revs_per_sec = speed/(self.traction_factor * self.wheel_circumference)
        req_steps_per_sec = req_revs_per_sec * self.steps_per_rev * (1/self.stepping_mode) # PWM freq

        self.setup_timer("Turn Timer", 1, self.timer_function) 
        self.pi.hardware_PWM(self.stepL, int(req_steps_per_sec), 500000)
        self.pi.hardware_PWM(self.stepR, int(req_steps_per_sec), 500000)

    def turn_right(self, speed=1):
        """ Function to make the robot turn right """
        # Reverse the left motor and keep the right motor forward
        GPIO.output(self.dirL, GPIO.LOW)   # Reverse left motor
        GPIO.output(self.dirR, GPIO.LOW)

        req_revs_per_sec = speed/(self.traction_factor * self.wheel_circumference)
        req_steps_per_sec = req_revs_per_sec * self.steps_per_rev * (1/self.stepping_mode) # PWM freq

        self.setup_timer("Turn Timer", 1, self.timer_function)
        self.pi.hardware_PWM(self.stepL, int(req_steps_per_sec), 500000)
        self.pi.hardware_PWM(self.stepR, int(req_steps_per_sec), 500000)

    def setup_timer(self, name, duration_seconds, function):
        """Sets up a timer to execute a function after a certain duration."""
        def timer_thread():
            print(f"{name} timer started at {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
            sleep(duration_seconds)
            function(name)

        thread = threading.Thread(target=timer_thread)
        thread.start()

    def timer_function(self, name):
        """Function to be executed when the timer expires."""
        self.pi.hardware_PWM(self.stepL, 0, 500000)
        self.pi.hardware_PWM(self.stepR, 0, 500000)
        print(f"{name} timer expired at {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")

   
if __name__ == "__main__":
    
    try:
        robot = Drive()
        robot.set_1D_direction(dirForward=True)
        robot.drive(speed=0.2)

        # Example of turning left and right
        sleep(2)
        robot.turn_left(speed=0.1)
        sleep(2)
        robot.turn_right(speed=0.1)
        # while True:
        #     pass

    except KeyboardInterrupt:
        # terminate gracefully
        robot.pi.hardware_PWM(robot.stepL, 0, 500000)
        robot.pi.hardware_PWM(robot.stepR, 0, 500000)
        GPIO.cleanup()
        sys.exit()