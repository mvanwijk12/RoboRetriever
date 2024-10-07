# This is the main code
import time
# from drive import Drive
from drive_B import Drive_B
from Pcontrol import Controller
from server import ConnectionServer
import logging
import logging.config
import numpy as np

# Constants
LIMIT_SWITCH_PIN = 10
turn_now = 3

class ItemsStack:
    def __init__(self, maxlen=10):
        self.stack = []
        self.maxlen = maxlen

    def push(self, item):
        """ Push a new value onto stack """
        self.stack.append(item)
        if len(self.stack) > self.maxlen:
            self.stack.pop(0)  # Remove the oldest item if size exceeds maxlen

    def is_empty(self):
        """ Check if stack is empty """
        return len(self.stack) == 0

    def pop(self):
        """ Pop off last value """
        return self.stack.pop(-1)

    def get_items(self):
        """ Return stack as a list """
        return self.stack


class RobotController:
    MAX_BALLS = 5
    MAX_TASK_TIME_S = 7 * 60
    FAN_TURN_ON_THRES_DISTANCE = 2 # in metres

    def __init__(self):
        self.stored_pathway = ItemsStack(maxlen=10) # store the last 10 movements
        self.lwheel_sum = 0
        self.rwheel_sum = 0
        self.angle_line = None
        self.start_time = time.time()
        # self.lwheel = 0.0
        # self.rwheel = 0.0
        self.distance_step = 0.2 # in metres
        self.speed = 0.1 # in m/s
        self.con = ConnectionServer().start()
        self.stop = 'False'
        self.logger = logging.getLogger(__name__)
        self.counter = 0
        self.robot = Drive_B()
        self.controller = Controller(0.4, 0, 0) # value adjusted to deal with normalised error
        self.task_start_time = time.time()
    

    def state_function_map(self, state):
        """ Maps the system states to a function to execute the state.
         
        Parameters
        - state: system state as an integer between 0 and 16 (inclusive) 

        Returns
        - A function handle to that will execute the current system state
        """

        STATE_FUNCTION_MAP_LIST = [self.search_pattern, self.navigate_around_box, self.turn_away_from_line, self.turn_away_from_line, 
                            self.drive_towards_ball, self.drive_towards_ball, self.turn_away_from_line, self.turn_away_from_line, 
                            self.search_pattern, self.drive_towards_box, self.turn_away_from_line, self.drive_towards_box, 
                            self.search_pattern, self.drive_towards_box, self.turn_away_from_line, self.drive_towards_box, self.deposition]
        
        return STATE_FUNCTION_MAP_LIST[state]


    def search_pattern(self, **kwargs):
        """ Function to execute the search pattern state. Used to navigate through the tennis
         court quadrant efficiently and effectively. """
        
        # Handle kwargs
        if 'counter_thres' in kwargs:
            counter_thres = kwargs['counter_thres']
        else:
            counter_thres = 3 # default value

        if 'counter_wrap_around' in kwargs:
            counter_wrap_around = kwargs['counter_wrap_around']
        else:
            counter_wrap_around = 10 # default value


        if self.counter > counter_thres:
            # turn right
            lwheel = 0.1 # cannot be zero, for divide by zero reasons
            rwheel = 1
            self.counter = (self.counter + 1) % counter_wrap_around
        else:
            # drive straight
            lwheel = 1
            rwheel = 1
            self.counter = (self.counter + 1) % counter_wrap_around

        # Save the multiplier for both wheels together, used for reversing
        self.stored_pathway.push([lwheel, rwheel])

        self.logger.info(f'forwards: left wheel {round(lwheel, 2)} right wheel {round(rwheel, 2)}')
        self.robot.relative_drive(distance=self.distance_step, speed=self.speed, scaling_L=lwheel, scaling_R=rwheel) 


    def drive_towards_ball(self, **kwargs):
        """ Function to execute the drive towards ball state. Used to navigate the robot towards
         a tennis ball in the quadrant. """
        if 'inf_results' in kwargs:
            tennis_ball_bbox = kwargs['inf_results'][0]['box']
        else:
            raise Exception('Inference results not passed to self.drive_toward_ball')
        
        normalised_pixel_error = 1/2*(tennis_ball_bbox['x1'] + tennis_ball_bbox['x2']) - 1/2
        self.logger.info("Ball detected, control initiated to traverse to ball ...")
        self.counter = 0
        PIDout = self.controller.PID(normalised_pixel_error)
        lwheel, rwheel = self.controller.homing_multiplier(PIDout)
        dist = self.estimate_distance(tennis_ball_bbox)
        if dist <= self.FAN_TURN_ON_THRES_DISTANCE:
            self.robot.fan_ctrl(on=True)

        # Save the multiplier for both wheels together, used for reversing
        self.stored_pathway.push([lwheel, rwheel])

        self.logger.info(f'forwards: left wheel {round(lwheel, 2)} right wheel {round(rwheel, 2)}')
        self.robot.relative_drive(distance=self.distance_step, speed=self.speed, scaling_L=lwheel, scaling_R=rwheel) 


    def turn_away_from_line(self, **kwargs):
        """ Function to execute the turn away from line state. Used to navigate the robot away
         from the court quadrant boundaries. """
        if 'inf_results' in kwargs:
            direction = np.array(kwargs['inf_results'][2]['line_direction'])
            self.robot.rebound_off_line(direction)
        else:
            raise Exception('Inference results not passed to self.turn_away_from_line')


    def drive_towards_box(self, **kwargs):
        """ Function to execute the drive towards box state. Used to navigate the robot towards
         the deposition box in the quadrant. """
        if 'inf_results' in kwargs:
            box_bbox = kwargs['inf_results'][1]['box']
        else:
            raise Exception('Inference results not passed to self.drive_towards_box')
        
        normalised_pixel_error = 1/2*(box_bbox['x1'] + box_bbox['x2']) - 1/2
        self.logger.info("Box detected, control initiated to traverse to box...")
        self.counter = 0
        PIDout = self.controller.PID(normalised_pixel_error)
        lwheel, rwheel = self.controller.homing_multiplier(PIDout)

        # Save the multiplier for both wheels together, used for reversing
        self.stored_pathway.push([lwheel, rwheel])

        self.logger.info(f'forwards: left wheel {round(lwheel, 2)} right wheel {round(rwheel, 2)}')
        self.robot.relative_drive(distance=self.distance_step, speed=self.speed, scaling_L=lwheel, scaling_R=rwheel) 


    def navigate_around_box(self, **kwargs):
        """ Function to execute the drive around box state. Used to navigate the robot away from
         the deposition box in the quadrant.  """
        if 'inf_results' in kwargs:
            box_bbox = kwargs['inf_results'][1]['box']
        else:
            raise Exception('Inference results not passed to self.drive_towards_box')
        
        normalised_pixel_error = 1/2*(box_bbox['x1'] + box_bbox['x2']) - 1/2
        self.logger.info("Box detected, control initiated to avoid box ...")
        self.counter = 0
        PIDout = self.controller.PID(normalised_pixel_error)
        lwheel, rwheel = self.controller.homing_multiplier(PIDout)

        # Take reciprocal of wheel velocity to avoid box
        lwheel = 1/lwheel
        rwheel = 1/rwheel

        # Save the multiplier for both wheels together, used for reversing
        self.stored_pathway.push([lwheel, rwheel])

        self.logger.info(f'forwards: left wheel {round(lwheel, 2)} right wheel {round(rwheel, 2)}')
        self.robot.relative_drive(distance=self.distance_step, speed=self.speed, scaling_L=lwheel, scaling_R=rwheel) 


    def deposition(self):
        """ Function to execute the ball deposition state. Used to release the balls into the 
         deposition box and setup the robot for the next round of ball collection. """

        # Open deposition lid
        self.robot.deposition_ctrl(open=True)
        time.sleep(5) # wait 5s for balls to drop out
        self.robot.shake(count=3)
        time.sleep(2)

        # Setup for next round of balls
        self.robot.n_collected_balls = 0
        self.robot.reached_box = False

        # Backtrack into quadrant
        while not self.stored_pathway.is_empty():
            lwheel, rwheel = self.stored_pathway.pop()
            self.logger.info(f'backwards: left wheel {round(lwheel, 2)} right wheel {round(rwheel, 2)}')
            self.robot.relative_drive(distance=-self.distance_step, speed=self.speed, scaling_L=lwheel, scaling_R=rwheel) 

    

    def parse_message(self, msg):
        """ Parses message sent from computer, filters for largest tennis ball and box (by bbox area).
        
        Parameters:
        - msg: msg object receieved from computer processing image stream

        Returns:
        - [tennis_ball_obj, box_obj, line_direction] where:
            - tennis_ball_obj: dictionary (or None) containing the inference results of the largest detected tennis ball
            - box_obj: dictionary (or None) containing the inference results of the largest detected box
            - line_direction: list (or None) of the form [a, b] where a*x + b*y = 0, specifying the direction of the unique dominant court line
        """
        A_max_tennis_ball = 0
        A_max_box = 0

        tennis_ball_obj = None
        box_obj = None
        line_direction = None
        inf_results = [None, None, None]

        for obj in msg:
            try:
                name = obj["name"]
                if name == 'tennis ball':
                    if self.bbox_area(obj["box"]) > A_max_tennis_ball:
                        A_max_tennis_ball = self.bbox_area(obj["box"])
                        tennis_ball_obj = obj
                else: # name == 'box'
                    if self.bbox_area(obj["box"]) > A_max_box:
                        A_max_box = self.bbox_area(obj["box"])
                        box_obj = obj
            except KeyError: # line_direction
                line_direction = obj["line_direction"]

        inf_results[0] = tennis_ball_obj
        inf_results[1] = box_obj
        inf_results[2] = line_direction

        return inf_results
            

    def bbox_area(self, box):
        """ Returns the area of the bbox """
        x1 = box["x1"]
        y1 = box["y1"]
        x2 = box["x2"]
        y2 = box["y2"]
        return abs(y2 - y1) * abs(x2 - x1)
    

    def determine_state(self, inf_results):
        """ Determines the state of the FSM. Takes as input the inference results and interrupt flags
        for task timer and limit switches. Must be called frequently to avoid missing fleeting states. 
        
        Parameters:
        - inf_results: inference results filtered for largest tennis ball and box (returned from self.parse_message)
        
        Returns:
        - System state as an integer between 0 and 16 (inclusive)
        """
        bit0 = inf_results[1] is not None
        bit1 = inf_results[2] is not None
        bit2 = inf_results[0] is not None
        bit3 = (time.time() - self.task_start_time >= self.MAX_TASK_TIME_S) or (self.robot.n_collected_balls >= self.MAX_BALLS)
        bit4 = self.robot.reached_box

        if not bit3: # if bit3 is 0, we don't care about bit4
            state = bit2 * 2**2 + bit1 * 2**1 + bit0 * 2**0
        else:
            # Turn off vacuum fan if stop condition is reached
            self.robot.fan_ctrl(on=False)

            if bit4:
                state = 16
            else:
                state = bit3 * 2**3 + bit2 * 2**2 + bit1 * 2**1 + bit0 * 2**0
        return state
    
    def estimate_distance(self, bbox, focal_pixel=770/1280, real_world_diameter=67e-3):
        """
        Estimate the distance of the tennis ball from the camera.

        Parameters:
        - bbox: dictionary {'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2} of the normalised bounding box coordinates.
        - focal_length: float, the focal length of the camera in pixels.
        - real_world_diameter: float, the real-world diameter of the tennis ball in m.

        Returns:
        - distance: float, estimated distance to the tennis ball in m.
        """
        # Extract bounding box width and height
        bbox_widths = abs(bbox['x2'] - bbox['x1'])

        # Calculate distance using the formula: Distance = (Real Diameter * Focal Length) / Perceived Diameter
        return np.where(bbox_widths != 0, (real_world_diameter * focal_pixel) / bbox_widths, 1e5)

    def main_loop(self):
        """ Entry point of program """
        while True:
            # Retrieve data from computer
            try:
                x = self.con.get_message()
                inf_results = self.parse_message(x)
                state = self.determine_state(inf_results)
                fn_hdle = self.state_function_map(state)
                fn_hdle(inf_results=inf_results)
            
            except: # catch all exceptions including KeyboardInterrupt
                # terminate gracefully
                self.robot.all_stop()
                self.con.close()
                raise Exception('Motors turned off - closing ...')

    def run(self):
        try:
            self.main_loop()
        finally:
            self.logger.info('system ended')
            # self.cleanup()


if __name__ == "__main__":
    logging.config.fileConfig('log.conf')
    logger = logging.getLogger(__name__)
    robot_controller = RobotController()
    robot_controller.run()
