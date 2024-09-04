"""
This is a python class to control driving
"""
__author__ = "Matt van Wijk, Alexander Jones"
__date__ = "05/09/2024"

# start the pigpiod daemon
import os
os.system("sudo pigpiod")
import math
import pigpio
import sys
import time
import RPi.GPIO as GPIO
from Pcontrol import Controller
import logging
import logging.config

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
        self.wheel_diameter_measured = 95e-3 * math.pi # measured wheel diameter 95mm
        self.dia_scaleL = 1 # scaling factor for left wheel diameter
        self.dia_scaleR = 1 # scaling factor for right wheel diameter
        self.traction_factor = 0.9
        self.steps_per_rev = 200
        self.stepping_mode = 1/8 # Assume 1/8 stepping
        self.wheel_spacing = 0.306 # Measured distance between centres of wheels in m
        self.spacing_scale = 1 # scaling factor for wheel spacing
        self.pi = pigpio.pi()
        self.start_time = None
        self.logger = logging.getLogger(__name__)

        # This may raise warnings about GPIO being set already, ignore
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.dirL, GPIO.OUT)
        GPIO.setup(self.dirR, GPIO.OUT)

    def set_1D_direction(self, dirForward=True):
        """ Set the drive direction as either forwards or backwards """
        if dirForward:
            GPIO.output(self.dirL, GPIO.HIGH)
            GPIO.output(self.dirR, GPIO.LOW)
            self.logger.info("Setting drive direction as forward")
        else:
            GPIO.output(self.dirL, GPIO.LOW)
            GPIO.output(self.dirR, GPIO.HIGH)
            self.logger.info("Setting drive direction as backwards")

    def drive(self, distance=0.2, speed=0.05, leftwheel_multilpier=1.0, rightwheel_multiplier=1.0):
        """ Function to start driving a specified distance in metres """
        req_revolutions = distance/(self.traction_factor * self.wheel_diameter)
        self.logger.debug(f'#Revolutions: {req_revolutions}')

        req_steps = req_revolutions * self.steps_per_rev * (1/self.stepping_mode)
        self.logger.debug(f'#Required steps: {req_steps}')
        
        req_revs_per_sec = speed/(self.traction_factor * self.wheel_diameter)
        self.logger.debug(f'Required revs per sec: {req_revs_per_sec}')

        req_steps_per_sec = req_revs_per_sec * self.steps_per_rev * (1/self.stepping_mode) # PWM freq
        self.logger.debug(f'PWM frequency: {int(req_steps_per_sec)}')

        # Steering
        # if the wheel is small, the PWM frquency will need to increase
        left_steps_per_sec = req_steps_per_sec * leftwheel_multilpier / self.dia_scaleL
        right_steps_per_sec = req_steps_per_sec  * rightwheel_multiplier / self.dia_scaleR
        
        drive_time = req_steps/(req_steps_per_sec)

        # ramp up driving from stop
        self.logger.debug(f'quarter original speed for 0.2s')
        self.pi.hardware_PWM(self.stepR, int(req_steps_per_sec/4), 500000)
        self.pi.hardware_PWM(self.stepL, int(req_steps_per_sec/4), 500000)
        self.start_time = time.time()
        while time.time() - self.start_time < 0.2:
            pass

        # bit faster
        self.logger.debug(f'half original speed for 0.2s')
        self.pi.hardware_PWM(self.stepL, int(req_steps_per_sec/2), 500000)
        self.pi.hardware_PWM(self.stepR, int(req_steps_per_sec/2), 500000)
        self.start_time = time.time()
        while time.time() - self.start_time < 0.2:
            pass
        # Start driving at desired speed
        self.logger.info(f'Drive time at desired speed: {drive_time}s')        
        self.start_time = time.time()
        self.pi.hardware_PWM(self.stepL, int(left_steps_per_sec), 500000)
        self.pi.hardware_PWM(self.stepR, int(right_steps_per_sec), 500000)
        while time.time() - self.start_time < drive_time:
            pass

        # Now drive slower for a bit
        self.logger.debug(f'Half original speed for 0.2s')
        self.pi.hardware_PWM(self.stepR, int(req_steps_per_sec/2), 500000)
        self.pi.hardware_PWM(self.stepL, int(req_steps_per_sec/2), 500000)
        self.start_time = time.time()
        while time.time() - self.start_time < 0.2:
            pass

        # even slower
        self.logger.debug(f'Quarter original speed for 0.2s')
        self.pi.hardware_PWM(self.stepL, int(req_steps_per_sec/4), 500000)
        self.pi.hardware_PWM(self.stepR, int(req_steps_per_sec/4), 500000)
        self.start_time = time.time()
        while time.time() - self.start_time < 0.2:
            pass
       
        # stop
        self.all_stop()

    def tank_turn(self, direction=1, amount=0.1, speed=0.05):
        req_revolutions = amount/(self.traction_factor * self.wheel_diameter)
        req_steps = req_revolutions * self.steps_per_rev * (1/self.stepping_mode)
        self.logger.debug(f'#Required steps: {req_steps}')
        
        req_revs_per_sec = speed/(self.traction_factor * self.wheel_diameter)
        self.logger.debug(f'Required revs per sec: {req_revs_per_sec}')

        req_steps_per_sec = req_revs_per_sec * self.steps_per_rev * (1/self.stepping_mode) # PWM freq
        self.logger.debug(f'PWM frequency: {int(req_steps_per_sec)}')

        drive_time = req_steps/(req_steps_per_sec)
        
        # turn right (clockwise)
        if direction == 1:
            GPIO.output(self.dirL, GPIO.HIGH)
            GPIO.output(self.dirR, GPIO.HIGH)

        else:
            GPIO.output(self.dirL, GPIO.LOW)
            GPIO.output(self.dirR, GPIO.LOW)
        # Steering
        left_steps_per_sec = req_steps_per_sec / self.dia_scaleL
        right_steps_per_sec = req_steps_per_sec / self.dia_scaleR
        self.start_time = time.time()
        self.pi.hardware_PWM(self.stepL, int(left_steps_per_sec), 500000)
        self.pi.hardware_PWM(self.stepR, int(right_steps_per_sec), 500000)
        while time.time() - self.start_time < drive_time:
            pass
        # stop
        self.all_stop()

    def turn_through_angle(self, turn_rate = 20, angle = 90):
        # Turn rate in deg/s
        # Angle is turn to the right, negative values for left turn
        distance_per_rev = math.pi * self.wheel_spacing * self.spacing_scale
        wheel_distance = angle/360 * distance_per_rev
        self.logger.debug(f'Wheel distance: {wheel_distance}')
        wheel_speed = turn_rate/360 * distance_per_rev
        self.logger.debug(f'Wheel speed: {wheel_speed}')
        if angle >= 0:
            self.logger.debug(f'Right turn requested')
            self.tank_turn(1, wheel_distance, wheel_speed)
        else:
            self.logger.debug(f'Left turn requested')
            self.tank_turn(0, wheel_distance, wheel_speed)
    
    def interpret_request(self, mode, param1, param2):
        # mode is 0 for straight drive, 1 for turn on spot
        # param1 is distance/angle, positive for forward/right
        # param2 is speed/rotation rate, always positive
        if mode == 0:
            if param1 >= 0:
                self.set_1D_direction(True)
            else:
                self.set_1D_direction(False)
            if param2 >0:
                self.logger.debug(f'Straight line distance: {param1}, speed: {param2}')
                self.drive(param1, param2, 1, 1)
        if mode == 1:
            if param2 > 0:
                self.logger.debug(f'Turn angle: {param1}, rate: {param2}')
                self.turn_through_angle(param2, param1)


    def all_stop(self):
        self.logger.debug(f'Stopping')
        self.pi.hardware_PWM(self.stepR, 0, 500000)
        self.pi.hardware_PWM(self.stepL, 0, 500000)

   
if __name__ == "__main__":
   logging.config.fileConfig('log.conf')
   logger = logging.getLogger(__name__)
   robot = Drive()
   while True: 
        try:
            print("Robot ready for driving")
            req_mode = input("Enter driving type, 0 for straight line, 1 for turn on spot: ")
            if req_mode == 0:
                req_param1 = input("Enter distance, negative for backwards driving: ")
                req_param2 = -1
                while req_param2 <= 0:
                    req_param2 = input("Enter speed in m/s: ")
                print("Caution: Robot about to move")
                time.sleep(1)
                robot.interpret_request(req_mode, req_param1, req_param2)

            if req_mode == 1:
                req_param1 = input("Enter angle to turn in degrees, negative for left turn: ")
                req_param2 = -1
                while req_param2 <= 0:
                    req_param2 = input("Enter turn rate in degrees/s: ")
                print("Caution: Robot about to move")
                time.sleep(1)
                robot.interpret_request(req_mode, req_param1, req_param2)


        except:
            # terminate gracefully
            robot.all_stop()
            GPIO.cleanup()
            sys.exit()
