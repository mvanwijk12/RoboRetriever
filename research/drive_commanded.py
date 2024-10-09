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
        self.wheel_diameter = 95e-3 * math.pi # measured wheel diameter 95mm
        self.traction_factor = 0.9
        self.steps_per_rev = 200
        self.stepping_mode = 1/8 # Assume 1/8 stepping
        self.wheel_spacing = 0.285 # Measured distance between inside contact points of wheels in m

        self.diameter_scaleL = 1 # scaling factor for left wheel diameter
        self.diameter_scaleR = 1 # scaling factor for right wheel diameter
        self.spacing_scale = 1 # scaling factor for wheel spacing

        self.speed_restrict = 0.2 # temporary speed restriction due to motor behaviour in week 7 lab

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
        self.logger.debug(f'Drive function called')
        if speed > self.speed_restrict:
            self.logger.info(f'Speed limited to {self.speed_restrict} m/s')
            speed = self.speed_restrict
            
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
        left_steps_per_sec = req_steps_per_sec * leftwheel_multilpier / self.diameter_scaleL
        right_steps_per_sec = req_steps_per_sec  * rightwheel_multiplier / self.diameter_scaleR
        
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
        #Drive time reduced by o.6s to account for distance travelled in acceleration and deceleration phases
        self.logger.info(f'Drive time at desired speed: {drive_time - 0.6}s')        
        self.start_time = time.time()
        self.pi.hardware_PWM(self.stepL, int(left_steps_per_sec), 500000)
        self.pi.hardware_PWM(self.stepR, int(right_steps_per_sec), 500000)
        while time.time() - self.start_time < drive_time - 0.6:
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
        self.logger.debug(f'Turn function called')
        if speed > self.speed_restrict:
            self.logger.info(f'Wheel speed limited to {self.speed_restrict} m/s')
            speed = self.speed_restrict
            
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
        left_steps_per_sec = req_steps_per_sec / self.diameter_scaleL
        right_steps_per_sec = req_steps_per_sec / self.diameter_scaleR
        self.start_time = time.time()
        self.pi.hardware_PWM(self.stepL, int(left_steps_per_sec), 500000)
        self.pi.hardware_PWM(self.stepR, int(right_steps_per_sec), 500000)
        while time.time() - self.start_time < drive_time:
            pass
        # stop
        self.all_stop()

    def turn_through_angle(self, turn_rate = 10, angle = 90):
        # Turn rate in deg/s
        # Angle is turn to the right, negative values for left turn
        distance_per_360 = math.pi * self.wheel_spacing * self.spacing_scale
        wheel_distance = angle/360 * distance_per_360
        self.logger.debug(f'Wheel distance: {wheel_distance}')
        wheel_speed = turn_rate/360 * distance_per_360
        self.logger.debug(f'Wheel speed: {wheel_speed}')
        if angle >= 0:
            self.logger.debug(f'Right turn request')
            self.tank_turn(1, wheel_distance, wheel_speed)
        else:
            self.logger.debug(f'Left turn request')
            self.tank_turn(0, -1 * wheel_distance, wheel_speed)
    
    def interpret_request(self, mode, param1, param2):
        # mode is 0 for straight drive, 1 for turn on spot
        # param1 is distance/angle, positive for forward/right
        # param2 is speed/rotation rate, always positive
        if mode == 0:
            if param2 > 0:
                if param1 >= 0:
                    self.logger.debug(f'Straight line forward: {param1}, speed: {param2}')
                    self.set_1D_direction(False) # Drive direction reversed
                    self.drive(param1, param2, 1, 1)
                else:
                    self.logger.debug(f'Straight line backward: {-1 * param1}, speed: {param2}')
                    self.set_1D_direction(True) # Drive direction reversed
                    self.drive(-1 * param1, param2, 1, 1)

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
   bench_mode = True # bench mode limits speed and distance to 0.1m, 0.05m/s, 360 deg, 10deg/s
   set_bench_mode = int(input("Robot is in bench mode by default, enter 1 to enable high speed, enter 0 to remain in bench mode: "))
   if set_bench_mode == 1:
       bench_mode = False
       robot.logger.info(f'Bench mode disabled')

   if bench_mode:
       robot.logger.info(f'Bench mode enabled, speed and distance constrained')
   
   while True: 
        try:
            print("Robot ready for driving")
            req_mode = int(input("Enter driving type, 0 for straight line, 1 for turn on spot: "))
            if req_mode == 0:
                distance = float(input("Enter distance, negative for backwards driving: "))
                speed = -1
                while speed <= 0:
                    speed = float(input("Enter speed in m/s: "))
                robot.logger.debug(f'Speed received as {speed}')

                if bench_mode:
                    if speed > 0.05:
                        speed = 0.05
                        print("Speed limited to 0.05 m/s in bench mode")
                    if distance > 0.1:
                        distance = 0.1
                        print("Distance limited to 0.1 m in bench mode")
                    if distance < -0.1:
                        distance = -0.1
                        print("Distance limited to 0.1 m in bench mode")

                print("Caution: Robot about to move")
                time.sleep(1)
                robot.interpret_request(req_mode, distance, speed)


            if req_mode == 1:
                angle = float(input("Enter angle to turn in degrees, negative for left turn: "))
                rate = -1
                while rate <= 0:
                    rate = float(input("Enter turn rate in degrees/s: "))
                robot.logger.debug(f'Turn rate received as {rate}')

                if bench_mode:
                    if rate > 10:
                        rate = 10
                        print("Turn rate limited to 10 deg/s in bench mode")
                    if angle > 360:
                        angle = 360
                        print("Angle limited to 360 degrees in bench mode")
                    if angle < -360:
                        angle = -360
                        print("Angle limited to 360 degrees in bench mode")

                print("Caution: Robot about to move")
                time.sleep(1)
                robot.interpret_request(req_mode, angle, rate)


        except Exception as e:
            # terminate gracefully
            robot.logger.info(f'Exception: {e}')
            robot.all_stop()
            GPIO.cleanup()
            sys.exit()
