"""
This is a revised python class to control driving
"""
__author__ = "Matt van Wijk, Alex Jones"
__date__ = "09/10/2024"

# start the pigpiod daemon
import os
os.system("sudo pigpiod")
import math
import pigpio
import sys
import time
import logging
import logging.config
import numpy as np


class Drive_B:
    """ Represents a robot drive object """
    def __init__(self, stepL=18, stepR=19, dirL=23, dirR=24, sucAcq=27, depoProx=25, fan=5, tach=6, act_retract=13, act_extend=12):
        """ Initalise the Drive Object
        
        Parameters
        - stepR: PWM pin connected to the right stepper motor driver step pin
        - stepL: PWM pin connected to the left stepper motor driver step pin
        - dirR: GPIO pin connected to the right stepper motor driver direction pin
        - dirL: GPIO pin connected to the left stepper motor driver direction pin
        - sucAcq: GPIO pin connected to the limit switch which triggers when a successful tennis ball acquision is made
        - depoProx: GPIO pin connected to the limit switch which triggers when the robot is in close proximity of the deposition box 
        """
        # Corrected assignment of direction pins as these had been reversed
        self.stepL = stepL
        self.stepR = stepR
        self.dirL = dirL
        self.dirR = dirR
        self.sucAcq = sucAcq
        self.depoProx = depoProx
        self.fan = fan
        self.tach = tach
        self.act_retract = act_retract
        self.act_extend = act_extend
        self.n_collected_balls = 0
        self.reached_box = False
        self.prev_time = time.time()
        self.current_speed_L = 0
        self.current_speed_R = 0


        # Set parameters for the robot
        self.wheel_diameter = 95e-3 * math.pi # measured wheel diameter 95mm
        self.traction_factor = 1
        self.steps_per_rev = 200
        self.stepping_mode = 1/8 # Assume 1/8 stepping
        self.wheel_spacing = 0.230 # Measured distance between inside contact points of wheels in m

        # Scaling factors to calibrate the driving accuracy
        self.diameter_scaleL = 1 # scaling factor for left wheel diameter, is multiplied
        self.diameter_scaleR = 1 # scaling factor for right wheel diameter, is multiplied
        self.spacing_scale = 1.034 # scaling factor for wheel spacing

        # Speed and acceleration parameters
        self.speed_restrict = 0.3 
        self.turn_rate_retrict = 45 # Turn rate limit in deg/s
        self.acceleration_rate = 0.05 # value in ms^-2
        self.acceleration_time_step = 0.02 # number of seconds between speed updates

        self.start_time = None
        self.logger = logging.getLogger(__name__)
        self.pi = pigpio.pi()
        if not self.pi.connected:
            self.logger.error(f'pigpio not initialised correctly!')
            exit()

        # This may raise warnings about GPIO being set already, ignore
        self.pi.set_mode(self.dirL, pigpio.OUTPUT)
        self.pi.set_mode(self.dirR, pigpio.OUTPUT)
        self.pi.set_mode(self.sucAcq, pigpio.INPUT)
        self.pi.set_mode(self.depoProx, pigpio.INPUT)

        # Configure pull-ups
        self.pi.set_pull_up_down(self.sucAcq, pigpio.PUD_UP)
        self.pi.set_pull_up_down(self.depoProx, pigpio.PUD_UP)

        # Set up interrupt for rising edges on the tach pin
        self.pi.callback(self.sucAcq, pigpio.FALLING_EDGE, self.successful_acquision_callback)
        self.pi.callback(self.depoProx, pigpio.FALLING_EDGE, self.box_proximity_callback)

    def successful_acquision_callback(self, gpio, level, tick):
        """ Callback function for when a tennis ball is collected successfully """
        cur_time = time.time()
        if cur_time - self.prev_time > 1:
            self.n_collected_balls += 1
            self.prev_time = cur_time
            # self.fan_ctrl(on=False) # Turn off vacuum fan
            self.logger.info(f'BALL COLLECTED! CURRENT BALL COUNT {self.n_collected_balls}')

    def box_proximity_callback(self, gpio, level, tick):
        """ Callback function for when limit switch hits the deposition box """
        self.reached_box = True 
        self.logger.info('BOX PROXIMITY LIMIT SWITCH TRIGGERED!')

    def fan_ctrl(self, on=True):
        """ Turns on/off the fan for the vacuum. """
        if on:
            self.pi.write(self.fan, pigpio.HIGH)
        else:
            self.pi.write(self.fan, pigpio.LOW)
    
    def deposition_ctrl(self, open=True):
        """ Opens/closes the deposition lid for depositing tennis balls. """
        if open:
            self.pi.write(self.act_retract, pigpio.HIGH)
            self.pi.write(self.act_extend, pigpio.LOW)
        else:
            self.pi.write(self.act_retract, pigpio.LOW)
            self.pi.write(self.act_extend, pigpio.HIGH)

    def _reflect_line_to_angle(self, rline):
        """ Converts the reflected line in the form a*x + b*y = 0 to a turn angle for the robot to move assuming optical dynamics. 
        
        :param rline: reflected line equation as a numpy array [a, b] where a*x + b*y = 0
        :returns: angle in degrees for the robot to turn in the rightward direction, negative angle corresponds to a leftward turn 
        """
        return math.atan2(rline[1], -rline[0]) * 180/math.pi
    
    def _reflection_line(self, line):
        """ Calculates the equation of the reflected line for an incidence direction of (x, y) = (0, 1) and a mirror line given by line.
        The reflected line is calculated assuming optical dynamics.
        
        :param line: mirror line equation as a numpy array [a, b] where a*x + b*y = 0
        :returns: reflected line equation as a numpy array [c, d] where c*x + d*y = 0
        """
        incidence_direction = np.array([1,0])
        normalized_mirror_direction = line/np.linalg.norm(line)
        reflected_direction = incidence_direction - (2*np.dot(incidence_direction, normalized_mirror_direction))*normalized_mirror_direction
        return reflected_direction/np.linalg.norm(reflected_direction) # TODO: check if reflected_direction can ever be 0

    def all_stop(self):
        """ Sets wheel velocities to zero """
        self.logger.debug(f'Stopping')
        self.pi.hardware_PWM(self.stepR, 0, 500000)
        self.pi.hardware_PWM(self.stepL, 0, 500000)
        
    def delete(self):
        """ Safely cleans up GPIO ports, to be called after self.all_stop() """
        self.pi.stop()




    def drive_wheels_at_speed(self, new_speed_L, new_speed_R, old_speed_L, old_speed_R):
        """Sets the drive directions and PWM rates for the given wheel speeds, no acceleration
        Negative speeds indicate backwards motion. The old speed is used to determine whether to set the direction."""
        # Last resort check of speed value
        assert abs(new_speed_L) < 2 * self.speed_restrict
        assert abs(new_speed_R) < 2 * self.speed_restrict

        # Multiply new and old speeds, if zero or negative the directions need to be set
        if new_speed_L * old_speed_L <= 0 or new_speed_R * old_speed_R <= 0:
            # Set wheel directions
            if new_speed_L < 0:
                self.pi.write(self.dirL, pigpio.LOW)
                self.logger.info('Direction changed, left wheel backward')
            else:
                self.pi.write(self.dirL, pigpio.HIGH)
                self.logger.info('Direction changed, left wheel forward')
            if new_speed_R < 0:
                self.pi.write(self.dirR, pigpio.HIGH)
                self.logger.info('Direction changed, right wheel backward')
            else:
                self.pi.write(self.dirR, pigpio.LOW)
                self.logger.info('Direction changed, right wheel forward')
        
        # Absolute value speeds, to avoid negative PWN frequencies
        new_speed_L = abs(new_speed_L)
        new_speed_R = abs(new_speed_R)

        # PWM frequency calculationassert self.traction_factor * self.diameter_scaleL * self.wheel_diameter != 0
        assert self.traction_factor * self.diameter_scaleR * self.wheel_diameter != 0
        assert self.stepping_mode != 0

        rev_rate_L = new_speed_L / (self.traction_factor * self.diameter_scaleL * self.wheel_diameter)
        rev_rate_R = new_speed_R / (self.traction_factor * self.diameter_scaleL * self.wheel_diameter)
        step_rate_L = rev_rate_L * self.steps_per_rev * (1/self.stepping_mode)
        step_rate_R = rev_rate_R * self.steps_per_rev * (1/self.stepping_mode)

        # Hardware PWM
        self.pi.hardware_PWM(self.stepL, int(step_rate_L), 500000)
        self.pi.hardware_PWM(self.stepR, int(step_rate_R), 500000)

    def calculate_limits(self, speed_L, speed_R):
        """ Function takes in wheel speeds and returns wheel speeds incorporating the linear and angular speed limits"""
        # Negative wheel speeds are accepted and should be supplied for sharp turns

        # Speed limiting
        assert self.speed_restrict > 0

        if max(abs(speed_L), abs(speed_R)) > self.speed_restrict:
            scale_by_linear_limit = self.speed_restrict / max(abs(speed_L), abs(speed_R))
        else: scale_by_linear_limit = 1

        # Turning rate limiting
        # The robot turns if one wheel travels further than the other
        distance_difference_per_360 = 2 * math.pi * self.wheel_spacing / self.spacing_scale
        max_speed_difference = self.turn_rate_retrict * distance_difference_per_360 / 360

        assert max_speed_difference != 0

        if abs(speed_L - speed_R) > max_speed_difference:
            scale_by_turn_limit = abs(max_speed_difference) / abs(speed_L - speed_R)
        else:
            scale_by_turn_limit = 1
        
        # Apply limit that results in lowest speed
        self.logger.debug(f'Limiting factors: linear speed: {round(scale_by_linear_limit, 2)}, turn_rate: {round(scale_by_turn_limit, 2)}')
        return speed_L * min(scale_by_linear_limit, scale_by_turn_limit), speed_R * min(scale_by_linear_limit, scale_by_turn_limit)

    def execute_drive(self, speed_L, speed_R):
        """Function to achieve a target speed
        Smooth acceleration to the speed, then leaves the robot moving at constant speed
        Blocking during the acceleration period
        Negative speeds indicate backwards movement"""

        threshold = self.acceleration_rate * self.acceleration_time_step

        # Calculate error and required acceleration rates
        speed_L, speed_R = self.calculate_limits(speed_L, speed_R)
        error_L = speed_L - self.current_speed_L
        error_R = speed_R - self.current_speed_R
        accel_time = max(abs(error_L), abs(error_R)) / self.acceleration_rate
        if accel_time != 0:
            accel_L = error_L / accel_time
            accel_R = error_R / accel_time

        self.logger.debug(f'Accelerating to speeds: left: {speed_L}, right: {speed_R} in {accel_time} s')
        # Acceleration loop
        while max(abs(error_L), abs(error_R)) > threshold:
            next_speed_L = self.current_speed_L + accel_L * self.acceleration_time_step
            next_speed_R = self.current_speed_R + accel_R * self.acceleration_time_step
            self.drive_wheels_at_speed(next_speed_L, next_speed_R, self.current_speed_L, self.current_speed_R)
            self.current_speed_L = next_speed_L
            self.current_speed_R = next_speed_R
            error_L = speed_L - self.current_speed_L
            error_R = speed_R - self.current_speed_R
            time.sleep(self.acceleration_time_step)
        
        # Set constant speed at target
        self.drive_wheels_at_speed(speed_L, speed_R, self.current_speed_L, self.current_speed_R)
        self.current_speed_L = speed_L
        self.current_speed_R = speed_R
        self.logger.debug('Reached target speed')

    def _turn_to_angle(self, angle, turn_rate, radius):
        """ Function to follow a curve of specified radius for a specified angle
        Requires the robot to be stationary when called, will result in a stationary robot when finished
        
        :param angle: angle in degrees for the robot to turn in the rightward direction, negative angle corresponds to a leftward turn
        :param turn_rate: the angular velocity of the turn in degrees/second
        :param radius: the turning radius in metres (non-negative value)
        """
        # Radius in metres, currently unable to drive backwards
        # Negative angles for left turn, angle in degrees
        # Radius should be positive or zero
        # turn_rate is the angular velocity in degrees/sec

        # stop the robot as the angle calculation assumes initial speed of zero
        self.execute_drive(0, 0)

        assert self.spacing_scale != 0
        assert turn_rate != 0
        assert self.acceleration_rate != 0

        # Calculate the required wheel speeds for the desired turn rate
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

            # Calculate the acceleration time and subtract from the drive time
            higher_speed = max(abs(speed_L), abs(speed_R))
            accel_time = higher_speed / self.acceleration_rate

            # Check if the intended speed is attainable
            if accel_time > drive_time:
                # calculate the speed after the higher distance wheel is halfway through the turn
                # v^2-u^2=2*a*s with u=0
                higher_dist = max(abs(distance_L), abs(distance_R))
                achievable_high_speed = math.sqrt(self.acceleration_rate * higher_dist) # intending half distance, so factor of 2 is present
                if abs(speed_L) > abs(speed_R):
                    speed_R = (speed_R / abs(speed_R)) * achievable_high_speed * speed_R / speed_L
                    speed_L = (speed_L / abs(speed_L)) * achievable_high_speed
                else:
                    speed_L = (speed_L / abs(speed_L)) * achievable_high_speed * speed_R / speed_L
                    speed_R = (speed_R / abs(speed_R)) * achievable_high_speed
                drive_time = 0

            else:
                # Robot average speed is half during acceleration, but there are two acceleration periods
                drive_time = drive_time - accel_time

            self.logger.info(f'Turning {round(angle, 2)}degrees at {round(turn_rate, 2)}degrees/second by driving {round(average_distance, 2)}m around a {round(radius, 2)}m circle')
            self.logger.debug(f'Wheel speeds are left: {round(speed_L, 2)}, right: {round(speed_R, 2)}')

            # Execute the turn
            self.execute_drive(speed_L, speed_R)
            time.sleep(drive_time)
            self.execute_drive(0, 0)
    
    def rebound_off_line(self, line, turn_rate=20, turn_radius=0.1):
        """ Takes in an equation of the mirror line and executes a reflected turn assuming optical dynamics.
         
        :param line: mirror line equation as a numpy array [a, b] where a*x + b*y = 0
        :param turn_rate: the angular velocity of the turn in degrees/second
        :param turn_radius: the turning radius in metres (non-negative value)
        """
        # Calculate reflected direction
        reflect_dir = self._reflection_line(line)
        self.logger.debug(f'Reflected line is {round(reflect_dir[0], 2)}x + {round(reflect_dir[1], 2)}y = 0')

        # Calculate turn angle
        turn_angle = self._reflect_line_to_angle(reflect_dir)

        # Execute turn
        self._turn_to_angle(turn_angle, turn_rate, turn_radius)



    def shake(self, count=1):
        """ Shakes the robot to help release stuck tennis balls in the pipes. """
        # The acceleration function calculates off the left wheel speed, so we can use the right wheel to avoid the acceleration time
        movement_time = 0.1
        right_wheel_speed = 0.1
        for _ in range(count):
            self.execute_drive(0, right_wheel_speed)
            time.sleep(movement_time)
            self.execute_drive(0, -1 * right_wheel_speed)
            time.sleep(movement_time)
            self.execute_drive(0, 0)


if __name__ == "__main__":
   logging.config.fileConfig('log.conf')
   logger = logging.getLogger(__name__)
   robot = Drive_B()

   while True: 
        try:
            print("Robot ready for driving")
            print("Hardware modes:")
            print("0: Driving steered as in Milestone 1")
            print("1: Constant radius with specified angle")
            print("2: Rebound off line")
            print("3: Fan control")
            print("4: Linear actuator control")
            print("5: Shake to release ball")

            req_mode = int(input("Enter driving mode: "))

            if req_mode == 0:
                time_at_speed = float(input("Enter time at constant speed, will be converted to positive: "))
                speed = float(input("Enter base speed in m/s: "))
                multiplier_L = float(input("Enter left wheel multiplier: "))
                multiplier_R = float(input("Enter right wheel multiplier: "))
                print("Caution: Robot about to move")
                time.sleep(1)

                distance = abs(distance)
                # Scale multipliers and base speed so that the higher multiplier has absolute value 1
                speed_L = speed * multiplier_L
                speed_R = speed * multiplier_R
                robot.execute_drive(speed_L, speed_R)
                time.sleep(time_at_speed)
                robot.execute_drive(0, 0)
            
            elif req_mode == 1:
                print("Currently unable to drive backwards in this mode")
                angle = float(input("Enter angle to turn, negtive values for left turn: "))
                turn_rate = float(input("Enter turn rate in degrees/s: "))
                radius = float(input("Enter radius in m, should be positive or zero: "))
                print("Caution: Robot about to move")
                time.sleep(1)
                robot._turn_to_angle(angle, turn_rate, radius)

            elif req_mode == 2:
                a = float(input("Enter line [a, b], a-value: "))
                b = float(input("Enter line [a, b], b-value: "))
                line = np.array([a, b])
                turn_rate = float(input("Enter turn rate/angular velocity (deg/s): "))
                turn_radius = float(input("Enter turn radius (m): "))
                print("Caution: Robot about to move")
                time.sleep(1)
                robot.rebound_off_line(line, turn_rate=turn_rate, turn_radius=turn_radius)

            elif req_mode == 3:
                selection = float(input("Enter 0 for fan off, anything else for fan on: "))
                if selection == 0:
                    robot.fan_ctrl(False)
                else:
                    robot.fan_ctrl(True)
            
            elif req_mode == 4:
                selection = float(input("Enter 0 for door closed, anything else for door open: "))
                if selection == 0:
                    robot.deposition_ctrl(False)
                else:
                    robot.deposition_ctrl(True)
            
            elif req_mode == 7:
                print("Performing hardware test")

                print("Left wheel forward")
                robot.execute_drive(0.1, 0)
                time.sleep(1)
                robot.execute_drive(0, 0)
                time.sleep(1)
                print("Right wheel forward")
                robot.relative_drive(0, 0.1)
                time.sleep(1)
                robot.execute_drive(0, 0)
                time.sleep(1)
                print("Left wheel backward")
                robot.relative_drive(-0.1, 0)
                time.sleep(1)
                robot.execute_drive(0, 0)
                time.sleep(1)
                print("Right wheel backward")
                robot.relative_drive(0, -0.1)
                time.sleep(1)
                robot.execute_drive(0, 0)
                time.sleep(1)

                print("Fan on")
                robot.fan_ctrl(True)
                time.sleep(10)
                print("Fan off")
                robot.fan_ctrl(False)
                time.sleep(3)

                print("Door open")
                robot.deposition_ctrl(True)
                time.sleep(3)
                print("Door close")
                robot.deposition_ctrl(False)
                time.sleep(3)

                print("Hardware test complete")

            elif req_mode == 5:
                shake_count = int(input("Enter number of shakes as positive integer: "))
                robot.shake(shake_count)

            else:
                print('Invalid selection')


        except Exception as e:
            # terminate gracefully
            robot.logger.info(f'Exception: {e}')
            robot.all_stop()
            robot.delete()
            sys.exit()

        except KeyboardInterrupt:
            # terminate gracefully
            robot.all_stop()
            robot.delete()
            sys.exit()

            
