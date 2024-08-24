__author__ = "Matt van Wijk"
__date__ = "13/08/2024"

# Modified by Gael to add turn
# Modified by Alex to request operator input

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
        self.wheel_spacing = 0.29 # Measured distance between centres of wheels in m
        self.drive_scaling = 1 # Scaling constant to correct forward/backward distance
        self.turn_scaling = 1  # Scaling constant to correct turn angle
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

    def turn_left(self, speed=1, duration=1):
        """ Function to make the robot turn left """
        # Reverse the right motor and keep the left motor forward
        GPIO.output(self.dirL, GPIO.HIGH)
        GPIO.output(self.dirR, GPIO.HIGH)  # Reverse right motor

        req_revs_per_sec = speed/(self.traction_factor * self.wheel_circumference)
        req_steps_per_sec = req_revs_per_sec * self.steps_per_rev * (1/self.stepping_mode) # PWM freq

        self.setup_timer("Turn Timer", duration, self.timer_function) 
        self.pi.hardware_PWM(self.stepL, int(req_steps_per_sec), 500000)
        self.pi.hardware_PWM(self.stepR, int(req_steps_per_sec), 500000)

    def turn_right(self, speed=1, duration=1):
        """ Function to make the robot turn right """
        # Reverse the left motor and keep the right motor forward
        GPIO.output(self.dirL, GPIO.LOW)   # Reverse left motor
        GPIO.output(self.dirR, GPIO.LOW)

        req_revs_per_sec = speed/(self.traction_factor * self.wheel_circumference)
        req_steps_per_sec = req_revs_per_sec * self.steps_per_rev * (1/self.stepping_mode) # PWM freq

        self.setup_timer("Turn Timer", duration, self.timer_function)
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

    def turn_through_angle(self, speed=1, angle=1, dir_left=True):
        """Function to perform a turn through a specified angle in degrees"""
        # Wheel speed in m/s
        # Rotation speed in rev/s
        rotation_rate = speed / (self.wheel_spacing * math.pi)
        # Turning code rotates both wheels at the same time, so effectively each wheel is driving around a radius of half the wheel spacing
        angle_revs = angle / 360
        turn_duration = angle_revs / rotation_rate
        if dir_left:
            self.turn_left(speed, turn_duration)
        else:
            self.turn_right(speed, turn_duration)



   
if __name__ == "__main__":
    
    try:
        robot = Drive()

        # Set the speed and rotation rate to use for drive commands
        linear_speed = 0.2  # m/s
        turning_speed = 0.1    # m/s wheel speed
        approx_turn_rate = 360/0.9 * turning_speed # Approximate, used for delaying input only
        
        # Robot uses both wheels to turn in a ~0.9m circumference around the centre of the robot

        while True:
            print("Robot ready for driving\n")
            req_mode = input("Enter driving type, 1 for straight line, 2 for turn on spot: ")

            if req_mode == "1":
                req_detail = input("Enter direction, 1 for forward, 2 for backward: ")

                if req_detail == "1":
                    robot.set_1D_direction(dirForward=True)
                    req_parameter = input("Enter desired distance in metres: ")
                    try:
                        req_dist = float(req_parameter)
                        print("Caution: Robot is moving\n")
                        sleep(1)
                        robot.drive(distance=req_dist * robot.drive_scaling, speed=linear_speed)
                        sleep(req_dist / linear_speed + 1)
                    except ValueError:
                        print("Invalid distance, returning to start of drive request")

                if req_detail == "2":
                    robot.set_1D_direction(dirForward=False)
                    req_parameter = input("Enter desired distance in metres: ")
                    try:
                        req_dist = float(req_parameter)
                        print("Caution: Robot is moving\n")
                        sleep(1)
                        robot.drive(distance=req_dist * robot.drive_scaling, speed=linear_speed)
                        sleep(req_dist / linear_speed + 1)
                    except ValueError:
                        print("Invalid distance, returning to start of drive request")
                else:
                    print("Enter parameter value as required, returning to start of drive request\n")

            if req_mode == "2":
                req_detail = input("Enter turn direction, 1 for left, 2 for right: ")

                if req_detail == "1":
                    req_parameter = input("Enter desired angle in degrees: ")
                    try:
                        req_angle = float(req_parameter)
                        print("Caution: Robot is moving\n")
                        sleep(1)
                        robot.turn_through_angle(speed=turning_speed,angle=req_angle * robot.turn_scaling,dir_left=True)
                        sleep(req_angle / approx_turn_rate + 1)
                    except ValueError:
                        print("Invalid angle, returning to start of drive request")
                    
                if req_detail == "2":
                    req_parameter = input("Enter desired angle in degrees: ")
                    try:
                        req_angle = float(req_parameter)
                        print("Caution: Robot is moving\n")
                        sleep(1)
                        robot.turn_through_angle(speed=turning_speed,angle=req_angle * robot.turn_scaling,dir_left=False)
                        sleep(req_angle / approx_turn_rate + 1)
                    except ValueError:
                        print("Invalid angle, returning to start of drive request")

                else:
                    print("Enter parameter value as required, returning to start of drive request\n")

            else:
                print("Enter parameter value as required, returning to start of drive request\n")

    except KeyboardInterrupt:
        # terminate gracefully
        robot.pi.hardware_PWM(robot.stepL, 0, 500000)
        robot.pi.hardware_PWM(robot.stepR, 0, 500000)
        GPIO.cleanup()
        sys.exit()
