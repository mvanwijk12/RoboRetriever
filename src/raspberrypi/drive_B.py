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


        # Set parameters for the robot
        self.wheel_diameter = 95e-3 * math.pi # measured wheel diameter 95mm
        self.traction_factor = 1
        self.steps_per_rev = 200
        self.stepping_mode = 1/8 # Assume 1/8 stepping
        self.wheel_spacing = 0.230 # Measured distance between inside contact points of wheels in m

        # Scaling factors to calibrate the driving accuracy
        self.diameter_scaleL = 1 # scaling factor for left wheel diameter
        self.diameter_scaleR = 1 # scaling factor for right wheel diameter
        self.spacing_scale = 1.034 # scaling factor for wheel spacing

        # Speed and acceleration parameters
        self.speed_restrict = 0.3 
        self.turn_rate_retrict = 45 # Turn rate limit in deg/s
        self.acceleration = 0.05 # value in ms^-2
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

    def all_stop(self):
        """ Sets wheel velocities to zero """
        self.logger.debug(f'Stopping')
        self.pi.hardware_PWM(self.stepR, 0, 500000)
        self.pi.hardware_PWM(self.stepL, 0, 500000)
        
    def delete(self):
        """ Safely cleans up GPIO ports, to be called after self.all_stop() """
        self.pi.stop()

    def set_wheel_direction(self, sideLeft=True, dirForward=True):
        """ Set the drive direction as either forwards or backwards for one of the wheels"""
        # The wheel direction reversal in Milestone 1 has been corrected,
        # so setting forward argument to true will result in forward movement
        # The wheel direction pin assignment correction required changing some values here
        if dirForward:
            if sideLeft:
                self.pi.write(self.dirL, pigpio.HIGH)
                self.logger.info('Setting drive direction, left wheel forward')
            else:
                self.pi.write(self.dirR, pigpio.LOW)
                self.logger.info('Setting drive direction, right wheel forward')
        else:
            if sideLeft:
                self.pi.write(self.dirL, pigpio.LOW)
                self.logger.info('Setting drive direction, left wheel backward')
            else:
                self.pi.write(self.dirR, pigpio.HIGH)
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
        assert self.traction_factor * self.diameter_scaleL * self.wheel_diameter != 0
        assert self.traction_factor * self.diameter_scaleR * self.wheel_diameter != 0
        assert self.stepping_mode != 0

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
        assert edge_speed_L != 0 
        assert edge_speed_R != 0

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
        
        self.logger.debug(f'Limiting factors: speed_L: {round(limit_by_speed_L, 2)}, speed_R: {round(limit_by_speed_R, 2)}, turn_rate: {round(limit_by_turn, 2)}')
        return (min(limit_by_speed_L, limit_by_speed_R, limit_by_turn))


    def execute_drive(self, drive_time=1, edge_speed_L=1, edge_speed_R=1):
        """ Function to drive each wheel at a specified speed for a specified time
        Accepts negative wheel speeds as a reverse instruction """
        # Drive time represents the time value used in the speed and distance calculation,
        # this function accounts for the acceleration curve it makes
        assert self.acceleration != 0
        assert edge_speed_L != 0
        assert edge_speed_R != 0 

        self.logger.info(f'Moving with speeds {round(edge_speed_L, 2)} and {round(edge_speed_R, 2)} for {round(drive_time, 2)} s')
        # Apply limits and adjust time to compensate if necessary
        limiting_factor = self.calculate_limits(edge_speed_L, edge_speed_R)
        self.logger.debug(f'Limiting factor applied: {round(limiting_factor, 2)}')
        
        assert limiting_factor != 0

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
        self.logger.debug(f'Drive direction set, speeds are now {round(edge_speed_L, 2)} and {round(edge_speed_R, 2)}')
        
        # Use the fastest edge wheel for the acceleration curve
        if edge_speed_L > edge_speed_R:
            acc_target_speed = edge_speed_L
            acc_left_controlled = True
            self.logger.debug(f'Using left wheel for acceleration curve')
        else:
            acc_target_speed = edge_speed_R
            acc_left_controlled = False
            self.logger.debug(f'Using right wheel for acceleration curve')

        # Estimates for acceleration and constant speed drive time
        acceleration_time = acc_target_speed / self.acceleration
        straight_time = drive_time - acceleration_time # The average speed in acceleration is half, but there are two time periods
        self.logger.debug(f'Accelerate for {round(acceleration_time, 2)}s and drive for {round(straight_time, 2)} s')

        # If no room, accelerate for half the distance then slow down
        if straight_time < 0:
            self.logger.debug(f'Not enough distance to accelerate to target speed')
            acc_total_distance = acc_target_speed * drive_time
            # Use v^2=2as for s being half the total distance
            acc_target_speed = math.sqrt(abs(self.acceleration * acc_total_distance))
            straight_time = 0

        # Constant acceleration loop
        speed_ratio = edge_speed_R / edge_speed_L # Always right/left
        # Apply the target acceleration rate to the fastest wheel, then use the ratio for the slower wheel
        acc_current_speed = 0
        current_speed_L = 0
        current_speed_R = 0
        self.logger.debug(f'Accelerating at {round(self.acceleration, 2)}m/s^2')
        while acc_current_speed < acc_target_speed:
            acc_current_speed += self.acceleration * self.acceleration_time_step
            if acc_left_controlled:
                current_speed_L = acc_current_speed
                current_speed_R = current_speed_L * speed_ratio
            else:
                current_speed_R = acc_current_speed
                current_speed_L = current_speed_R / speed_ratio
            
            step_rate_L = self.convert_speed_PWM_rate(current_speed_L, True)
            step_rate_R = self.convert_speed_PWM_rate(current_speed_R, False)
            self.set_drive_PWM(step_rate_L, step_rate_R, self.acceleration_time_step)

        # Constant acceleration allows the calculation that the distance travelled in accelerating
        # is half that that would be travelled at the target speed in the same time, allowing for the 
        # braking loop later means taking these values and subtracting them from the drive time

        # Drive most of the distance
        if straight_time > 0:
            self.logger.debug(f'Driving at constant speed for {round(straight_time, 2)} s')
            step_rate_L = self.convert_speed_PWM_rate(edge_speed_L, True)
            step_rate_R = self.convert_speed_PWM_rate(edge_speed_R, False)
            self.set_drive_PWM(step_rate_L, step_rate_R, straight_time)

        # Constant deceleration loop
        acc_current_speed = acc_target_speed
        
        self.logger.debug(f'Decelerating at {round(self.acceleration, 2)}m/s^2')
        while acc_current_speed > 0:
            acc_current_speed -= self.acceleration * self.acceleration_time_step
            if acc_left_controlled:
                current_speed_L = acc_current_speed
                current_speed_R = current_speed_L * speed_ratio
            else:
                current_speed_R = acc_current_speed
                current_speed_L = current_speed_R / speed_ratio

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
        self.logger.info(f'Driving straight {round(distance, 2)}m at {round(speed, 2)}m/s')
        if distance >= 0:
            self.execute_drive(drive_time, speed, speed)
        else:
            self.execute_drive(drive_time, -speed, -speed)


    def relative_drive(self, distance=1, speed=1, scaling_L=1, scaling_R=1):
        """ Function to drive the steering method from Milestone 1"""
        # Negative distance for backwards driving, speed is converted to absolute value
        # Speed limiting is applied later
        # Negative scaling is supported and drives that wheel in the other direction
        speed = abs(speed)
        drive_time = abs(distance / speed)

        # Scaling is converted so that the base speed is the same after scaling
        if scaling_L != -scaling_R:
            average_scaling = abs((scaling_L + scaling_R) / 2)
            scaling_L = scaling_L / average_scaling
            scaling_R = scaling_R / average_scaling
            # in other case,
            # the scalings are only of opposite sign, so the result should be that the base speed is the same

        self.logger.info(f'Driving {round(distance, 2)} m, left wheel speed: {round(speed * scaling_L, 2)}, right wheel speed: {round(speed * scaling_R, 2)}')
        if distance >= 0:
            self.execute_drive(drive_time, speed * scaling_L, speed * scaling_R)
        else:
            self.execute_drive(drive_time, -speed * scaling_L, -speed * scaling_R)


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
            self.logger.info(f'Driving {round(distance, 2)}m around arc of {round(radius, 2)}m radius')
        if distance >= 0:
            self.execute_drive(drive_time, speed * scaling_L, speed * scaling_R)
        else:
            self.execute_drive(drive_time, -speed * scaling_L, -speed * scaling_R)


    def _turn_to_angle(self, angle, turn_rate, radius):
        """ Function to follow a curve of specified radius for a specified angle
        
        :param angle: angle in degrees for the robot to turn in the rightward direction, negative angle corresponds to a leftward turn
        :param turn_rate: the angular velocity of the turn in degrees/second
        :param radius: the turning radius in metres (non-negative value)
        """
        # Radius in metres, currently unable to drive backwards
        # Negative angles for left turn, angle in degrees
        # Radius should be positive or zero
        # turn_rate is the angular velocity in degrees/sec
        assert self.spacing_scale != 0
        assert turn_rate != 0

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
            self.logger.info(f'Turning {round(angle, 2)}degrees at {round(turn_rate, 2)}degrees/second by driving {round(average_distance, 2)}m around a {round(radius, 2)}m circle')
            self.logger.debug(f'Wheel speeds are left: {round(speed_L, 2)}, right: {round(speed_R, 2)}')
            self.execute_drive(drive_time, speed_L, speed_R)


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

    def shake(self, count=1):
        """ Shakes the robot to help release stuck tennis balls in the pipes. """
        # The acceleration function calculates off the left wheel speed, so we can use the right wheel to avoid the acceleration time
        movement_time = 0.1
        right_wheel_speed = 0.15
        for _ in range(count):
            self.execute_drive(movement_time, 0.01, right_wheel_speed)
            time.sleep(0.1)
            self.execute_drive(movement_time, -0.01, -right_wheel_speed)
            time.sleep(0.1)


if __name__ == "__main__":
   logging.config.fileConfig('log.conf')
   logger = logging.getLogger(__name__)
   robot = Drive_B()

   while True: 
        try:
            print("Robot ready for driving")
            print("Hardware modes:")
            print("0: Straight line driving")
            print("1: Driving steered as in Milestone 1")
            print("2: Constant radius with specified arc length")
            print("3: Constant radius with specified angle")
            print("4: Rebound off line")
            print("5: Fan control")
            print("6: Linear actuator control")
            print("7: Hardware test")
            print("8: Shake to release ball")
            print("9: Quick deposit")

            req_mode = int(input("Enter driving mode: "))

            if req_mode == 0:
                distance = float(input("Enter distance, negative for backwards driving: "))
                speed = float(input("Enter speed in m/s: "))
                print("Caution: Robot about to move")
                time.sleep(1)
                robot.straight_drive(distance, speed)
            
            elif req_mode == 1:
                distance = float(input("Enter distance, negative for backwards driving: "))
                speed = float(input("Enter base speed in m/s: "))
                multiplier_L = float(input("Enter left wheel multiplier: "))
                multiplier_R = float(input("Enter right wheel multiplier: "))
                print("Caution: Robot about to move")
                time.sleep(1)
                robot.relative_drive(distance, speed, multiplier_L, multiplier_R)

            elif req_mode == 2:
                distance = float(input("Enter distance, negative for backwards driving: "))
                speed = float(input("Enter speed in m/s: "))
                radius = float(input("Enter radius in m, negative values for left turn: "))
                print("Caution: Robot about to move")
                time.sleep(1)
                robot.turn_to_distance(distance, speed, radius)
            
            elif req_mode == 3:
                print("Currently unable to drive backwards in this mode")
                angle = float(input("Enter angle to turn, negtive values for left turn: "))
                turn_rate = float(input("Enter turn rate in degrees/s: "))
                radius = float(input("Enter radius in m, should be positive or zero: "))
                print("Caution: Robot about to move")
                time.sleep(1)
                robot._turn_to_angle(angle, turn_rate, radius)

            elif req_mode == 4:
                a = float(input("Enter line [a, b], a-value: "))
                b = float(input("Enter line [a, b], b-value: "))
                line = np.array([a, b])
                turn_rate = float(input("Enter turn rate/angular velocity (deg/s): "))
                turn_radius = float(input("Enter turn radius (m): "))
                print("Caution: Robot about to move")
                time.sleep(1)
                robot.rebound_off_line(line, turn_rate=turn_rate, turn_radius=turn_radius)

            elif req_mode == 5:
                selection = float(input("Enter 0 for fan off, anything else for fan on: "))
                if selection == 0:
                    robot.fan_ctrl(False)
                else:
                    robot.fan_ctrl(True)
            
            elif req_mode == 6:
                selection = float(input("Enter 0 for door closed, anything else for door open: "))
                if selection == 0:
                    robot.deposition_ctrl(False)
                else:
                    robot.deposition_ctrl(True)
            
            elif req_mode == 7:
                print("Performing hardware test")

                print("Left wheel forward")
                robot.relative_drive(0.1, 0.1, 1, 0.01)
                time.sleep(1)
                print("Right wheel forward")
                robot.relative_drive(0.1, 0.1, 0.01, 1)
                time.sleep(1)
                print("Left wheel backward")
                robot.relative_drive(-0.1, 0.1, 1, 0.01)
                time.sleep(1)
                print("Right wheel backward")
                robot.relative_drive(-0.1, 0.1, 0.01, 1)
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

            elif req_mode == 8:
                shake_count = int(input("Enter number of shakes as positive integer: "))
                robot.shake(shake_count)

            elif req_mode == 9:
                robot.fan_ctrl(False)
                print("Waiting for fan to stop...")
                time.sleep(5)
                robot.deposition_ctrl(True)
                time.sleep(4)
                shake = input("Press ENTER to begin shake")
                robot.shake(5)
                complete = input("Press ENTER once all balls removed")
                robot.deposition_ctrl(False)
                robot.fan_ctrl(True)
                print("Complete")



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

            
