"""
This is a revised python class to control driving, based on the Milestone 1 drive class
"""
__author__ = "Matt van Wijk, Alex Jones"
__date__ = "12/09/2024"

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


class Drive_B:
    """ Represents a robot drive object """
    def __init__(self, stepL=18, stepR=19, dirL=23, dirR=24):
        """ Initalise the Drive Object
        
        :param stepR: PWM pin connected to the right stepper motor driver step pin
        :param stepL: PWM pin connected to the left stepper motor driver step pin
        :param dirR: GPIO pin connected to the right stepper motor driver direction pin
        :param dirL: GPIO pin connected to the left stepper motor driver direction pin
        """
        # Corrected assignment of direction pins as these had been reversed
        self.stepL = stepL
        self.stepR = stepR
        self.dirL = dirL
        self.dirR = dirR

        # Set parameters for the robot
        self.wheel_diameter = 95e-3 * math.pi # measured wheel diameter 95mm
        self.traction_factor = 1
        self.steps_per_rev = 200
        self.stepping_mode = 1/8 # Assume 1/8 stepping
        self.wheel_spacing = 0.287 # Measured distance between inside contact points of wheels in m

        # Scaling factors to calibrate the driving accuracy
        self.diameter_scaleL = 1 # scaling factor for left wheel diameter
        self.diameter_scaleR = 1 # scaling factor for right wheel diameter
        self.spacing_scale = 0.9839 # scaling factor for wheel spacing

        # Speed and acceleration parameters
        self.speed_restrict = 0.2 # speed restriction due to motor behaviour in week 7 lab
        self.turn_rate_retrict = 90 # Turn rate limit in deg/s
        self.acceleration = 0.1 # value in ms^-2
        self.acceleration_time_step = 0.02 # number of seconds between speed updates

        self.pi = pigpio.pi()
        self.start_time = None
        self.logger = logging.getLogger(__name__)

        # This may raise warnings about GPIO being set already, ignore
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.dirL, GPIO.OUT)
        GPIO.setup(self.dirR, GPIO.OUT)


    def all_stop(self):
        self.logger.debug(f'Stopping')
        self.pi.hardware_PWM(self.stepR, 0, 500000)
        self.pi.hardware_PWM(self.stepL, 0, 500000)


    def set_wheel_direction(self, sideLeft=True, dirForward=True):
        """ Set the drive direction as either forwards or backwards for one of the wheels"""
        # The wheel direction reversal in Milestone 1 has been corrected,
        # so setting forward argument to true will result in forward movement
        # The wheel direction pin assignment correction required changing some values here
        if dirForward:
            if sideLeft:
                GPIO.output(self.dirL, GPIO.HIGH)
                self.logger.info('Setting drive direction, left wheel forward')
            else:
                GPIO.output(self.dirR, GPIO.LOW)
                self.logger.info('Setting drive direction, right wheel forward')
        else:
            if sideLeft:
                GPIO.output(self.dirL, GPIO.LOW)
                self.logger.info('Setting drive direction, left wheel backward')
            else:
                GPIO.output(self.dirR, GPIO.HIGH)
                self.logger.info('Setting drive direction, right wheel backward')


    def set_drive_PWM(self, step_rate_L=1, step_rate_R=1, run_time=1):
        """ Perform the PWM operations to make a driving move"""
        # All arguments are positive

        # This causes a lot of debug messages, comment out if not needed
        # self.logger.debug(f'PWM left freq: {step_rate_L}, right freq:{step_rate_R} for {run_time} s"')

        self.pi.hardware_PWM(self.stepL, int(step_rate_L), 500000)
        self.pi.hardware_PWM(self.stepR, int(step_rate_R), 500000)
        self.start_time = time.time()
        while time.time() - self.start_time < run_time:
            pass


    def convert_speed_PWM_rate(self, edge_speed=1, sideLeft=True):
        """ Takes in the edge speed for one wheel and returns the corresponding PWM frequency"""
        # Edge speed is positive and already limited
        # Traction factor and diameter scaling are applied at this point
        if sideLeft:
            rev_rate = edge_speed / (self.traction_factor * self.diameter_scaleL * self.wheel_diameter)
        else:
            rev_rate = edge_speed / (self.traction_factor * self.diameter_scaleR * self.wheel_diameter)
        step_rate = rev_rate * self.steps_per_rev * (1/self.stepping_mode)
        if step_rate > 0:
            return step_rate
        else:
            return 0


    def calculate_limits(self, edge_speed_L=1, edge_speed_R=1):
        """ Function takes in wheel speeds and calculates a scaling factor to apply speed and turning limits
        Returns 1 if no speed limiting is needed"""
        # Negative wheel speeds are accepted and should be supplied for sharp turns
        # Speed limiting
        if abs(edge_speed_L) > self.speed_restrict:
            limit_by_speed_L = abs(self.speed_restrict / edge_speed_L)
        else: limit_by_speed_L = 1
        if abs(edge_speed_R) > self.speed_restrict:
            limit_by_speed_R = abs(self.speed_restrict / edge_speed_R)
        else: limit_by_speed_R = 1

        # Turning rate limiting
        # The robot turns if one wheel travels further than the other
        distance_difference_per_360 = 2 * math.pi * self.wheel_spacing / self.spacing_scale
        max_speed_difference = self.turn_rate_retrict * distance_difference_per_360 / 360
        if abs(edge_speed_L - edge_speed_R) > max_speed_difference:
            limit_by_turn = abs(max_speed_difference / abs(edge_speed_L - edge_speed_R))
        else:
            limit_by_turn = 1
        
        self.logger.debug(f'Limiting factors: speed_L: {limit_by_speed_L}, speed_R: {limit_by_speed_R}, turn_rate: {limit_by_turn}')
        return(min(limit_by_speed_L, limit_by_speed_R, limit_by_turn))


    def execute_drive(self, drive_time=1, edge_speed_L=1, edge_speed_R=1):
        """ Function to drive each wheel at a specified speed for a specified time
        Accepts negative wheel speeds as a reverse instruction """
        # Drive time represents the time value used in the speed and distance calculation,
        # this function accounts for the acceleration curve it makes

        self.logger.info(f'Moving with speeds {edge_speed_L} and {edge_speed_R} for {drive_time} s')
        # Apply limits and adjust time to compensate if necessary
        limiting_factor = self.calculate_limits(edge_speed_L, edge_speed_R)
        self.logger.debug(f'Limiting factor applied: {limiting_factor}')
        edge_speed_L = edge_speed_L * limiting_factor
        edge_speed_R = edge_speed_R * limiting_factor
        drive_time = drive_time / limiting_factor

        # Set wheel directions
        if edge_speed_L < 0:
            self.set_wheel_direction(True, False)
        else:
            self.set_wheel_direction(True, True)
        if edge_speed_R < 0:
            self.set_wheel_direction(False, False)
        else:
            self.set_wheel_direction(False, True)
        
        # Convert edge speeds to absolute values once the direction has been set
        edge_speed_L = abs(edge_speed_L)
        edge_speed_R = abs(edge_speed_R)
        self.logger.debug(f'Drive direction set, speeds are now {edge_speed_L} and {edge_speed_R}')
        
        # Estimates for acceleration and constant speed drive time
        acceleration_time = edge_speed_L / self.acceleration
        straight_time = drive_time - acceleration_time # The average speed in acceleration is half, but there are two time peroiods
        self.logger.debug(f'Accelerate for {acceleration_time} s and drive for {straight_time} s')

        # TODO: Apply the acceleration to the average speed of the robot, not to one wheel
        # TODO: Divide the distance test into three regions, slow moves over short distance and accelerate/decelerate for moderate distance

        # If no room, perform the drive slowly
        if straight_time < 0:
            self.logger.debug(f'Moving {edge_speed_L * drive_time} m slowly as no room to accelerate')
            reduction = 0.05 / edge_speed_L
            drive_time = drive_time / reduction
            edge_speed_L = 0.05
            edge_speed_R = reduction * edge_speed_R
            step_rate_L = self.convert_speed_PWM_rate(edge_speed_L, True)
            step_rate_R = self.convert_speed_PWM_rate(edge_speed_R, False)
            self.set_drive_PWM(step_rate_L, step_rate_R, drive_time)

            self.all_stop()

        else:
            # Constant acceleration loop
            speed_ratio = edge_speed_R / edge_speed_L
            # Apply the target acceleration rate to the left wheel speed
            current_speed_L = 0
            self.logger.debug(f'Accelerating at {self.acceleration} m/s^2')
            while current_speed_L < edge_speed_L:
                current_speed_L += self.acceleration * self.acceleration_time_step
                current_speed_R = current_speed_L * speed_ratio
                step_rate_L = self.convert_speed_PWM_rate(current_speed_L, True)
                step_rate_R = self.convert_speed_PWM_rate(current_speed_R, False)
                self.set_drive_PWM(step_rate_L, step_rate_R, self.acceleration_time_step)

            # Constant acceleration allows the calculation that the distance travelled in accelerating
            # is half that that would be travelled at the target speed in the same time, allowing for the 
            # braking loop later means taking these values and subtracting them from the drive time

            # Drive most of the distance
            self.logger.debug(f'Driving at constant speed for {straight_time} s')
            step_rate_L = self.convert_speed_PWM_rate(edge_speed_L, True)
            step_rate_R = self.convert_speed_PWM_rate(edge_speed_R, False)
            self.set_drive_PWM(step_rate_L, step_rate_R, straight_time)

            # Constant deceleration loop
            current_speed_L = edge_speed_L
            self.logger.debug(f'Decelerating at {self.acceleration} m/s^2')
            while current_speed_L > 0:
                current_speed_L -= self.acceleration * self.acceleration_time_step
                current_speed_R = current_speed_L * speed_ratio
                step_rate_L = self.convert_speed_PWM_rate(current_speed_L, True)
                step_rate_R = self.convert_speed_PWM_rate(current_speed_R, False)
                self.set_drive_PWM(step_rate_L, step_rate_R, self.acceleration_time_step)

            self.all_stop()


    def straight_drive(self, distance=1, speed=1):
        """ Function to drive a specified distance in metres at a specified speed"""
        # Negative distance for backwards driving, speed is converted to absolute value
        # Speed limiting is applied later
        speed = abs(speed)
        drive_time = abs(distance / speed)
        self.logger.info(f'Driving straight {distance} m at {speed} m/s')
        if distance >= 0:
            self.execute_drive(drive_time, speed, speed)
        else:
            self.execute_drive(drive_time, -1 * speed, -1 * speed)


    def relative_drive(self, distance=1, speed=1, scaling_L=1, scaling_R=1):
        """ Function to drive the steering method from Milestone 1"""
        # Negative distance for backwards driving, speed is converted to absolute value
        # Speed limiting is applied later
        # Negative scaling is supported and drives that wheel in the other direction
        speed = abs(speed)
        drive_time = abs(distance / speed)

        # Scaling is converted so that the base speed is the same after scaling
        if scaling_L != -1 * scaling_R:
            average_scaling = abs((scaling_L + scaling_R) / 2)
            scaling_L = scaling_L / average_scaling
            scaling_R = scaling_R / average_scaling
            # in other case,
            # the scalings are only of opposite sign, so the result should be that the base speed is the same

        self.logger.info(f'Driving {distance} m, left wheel speed: {speed * scaling_L}, right wheel speed: {speed * scaling_R}')
        if distance >= 0:
            self.execute_drive(drive_time, speed * scaling_L, speed * scaling_R)
        else:
            self.execute_drive(drive_time, -1 * speed * scaling_L, -1 * speed * scaling_R)


    def turn_to_distance(self, distance=1, speed=1, radius=1):
        """ Function to follow a curve of specified radius for a specified distance"""
        # Radius in metres, negative distance for backwards driving
        # Positive radius for right turn, negative radius for left turn
        # Radius is non-zero as there is a distance to be travelled
        speed = abs(speed)
        drive_time = abs(distance / speed)
        
        distance_difference_per_360 = 2 * math.pi * self.wheel_spacing / self.spacing_scale
        # For a given radius, 360 degrees must be met in 2*pi*radius
        if radius != 0:
            scaling_R = 1 - distance_difference_per_360 / (2 * 2 * math.pi * radius)
            scaling_L = 1 + distance_difference_per_360 / (2 * 2 * math.pi * radius)
            self.logger.info(f'Driving {distance} m around arc of {radius} m radius')
        if distance >= 0:
            self.execute_drive(drive_time, speed * scaling_L, speed * scaling_R)
        else:
            self.execute_drive(drive_time, -1 * speed * scaling_L, -1 * speed * scaling_R)


    def turn_to_angle(self, angle=1, turn_rate=1, radius=1):
        """ Function to follow a curve of specified radius for a specified angle"""
        # Radius in metres, currently unable to drive backwards
        # Negative angles for left turn
        # Radius should be positive or zero
        turn_rate = abs(turn_rate)
        radius = abs(radius)

        distance_difference_per_360 = 2 * math.pi * self.wheel_spacing / self.spacing_scale

        if angle != 0:
            average_distance = abs(2 * math.pi * radius * angle / 360)
            distance_L = average_distance + (angle * distance_difference_per_360) / (360 * 2)
            distance_R = average_distance - (angle * distance_difference_per_360) / (360 * 2)
            drive_time = abs(angle / turn_rate)
            speed_L = distance_L / drive_time
            speed_R = distance_R / drive_time
            self.logger.info(f'Turning {angle} degrees by driving {average_distance} m around a {radius} m circle')
            self.logger.debug(f'Wheel speeds are left: {speed_L}, right: {speed_R}')
            self.execute_drive(drive_time, speed_L, speed_R)



if __name__ == "__main__":
   logging.config.fileConfig('log.conf')
   logger = logging.getLogger(__name__)
   robot = Drive_B()

   while True: 
        try:
            print("Robot ready for driving")
            print("Driving modes:")
            print("0: Straight line driving")
            print("1: Driving steered as in Milestone 1")
            print("2: Constant radius with specified arc length")
            print("3: Constant radius with specified angle")
            req_mode = int(input("Enter driving mode: "))
            if req_mode == 0:
                distance = float(input("Enter distance, negative for backwards driving: "))
                speed = float(input("Enter speed in m/s: "))
                print("Caution: Robot about to move")
                time.sleep(1)
                robot.straight_drive(distance, speed)
            
            if req_mode == 1:
                distance = float(input("Enter distance, negative for backwards driving: "))
                speed = float(input("Enter base speed in m/s: "))
                multiplier_L = float(input("Enter left wheel multiplier: "))
                multiplier_R = float(input("Enter right wheel multiplier: "))
                print("Caution: Robot about to move")
                time.sleep(1)
                robot.relative_drive(distance, speed, multiplier_L, multiplier_R)

            if req_mode == 2:
                distance = float(input("Enter distance, negative for backwards driving: "))
                speed = float(input("Enter speed in m/s: "))
                radius = float(input("Enter radius in m, negative values for left turn: "))
                print("Caution: Robot about to move")
                time.sleep(1)
                robot.turn_to_distance(distance, speed, radius)
            
            if req_mode == 3:
                print("Currently unable to drive backwards in this mode")
                angle = float(input("Enter angle to turn, negtive values for left turn: "))
                turn_rate = float(input("Enter turn rate in degrees/s: "))
                radius = float(input("Enter radius in m, should be positive or zero: "))
                print("Caution: Robot about to move")
                time.sleep(1)
                robot.turn_to_angle(angle, turn_rate, radius)


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