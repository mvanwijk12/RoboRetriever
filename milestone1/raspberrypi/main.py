# This is the main code
import time
from drive import Drive
from Pcontrol import Controller
from server import ConnectionServer
import logging
import logging.config

# Constants
LIMIT_SWITCH_PIN = 10
turn_now = 3

class RobotController:
    #robot = Drive()
    def __init__(self):
        self.stored_pathway = []
        self.lwheel_sum = 0
        self.rwheel_sum = 0
        self.angle_line = None
        self.start_time = time.time()
        self.lwheel = 0.0
        self.rwheel = 0.0
        self.con = ConnectionServer().start()
        self.stop = 'False'
        self.logger = logging.getLogger(__name__)
        self.counter = 0
      

    def main_loop(self):
        robot = Drive()
        # self.start_time = time.time()
        # input to controller includes the gains kp, ki, kd. these can be adjusted for tuning
        controller = Controller(0.0004,0,0)
        pixel_error = 0.0
        PIDout = None
        
        # speed = 0.05 # for all.
        # distance = 0.2 # for all
        while True:
            # Fetch data from laptop
            try:
                x = self.con.get_message()
                if x is None:
                    self.logger.info("no ball detected, driving straight")
                    if self.counter > 3:
                        # turn right
                        self.lwheel = 0
                        self.rwheel = 1
                    else:
                        # drive straight
                        PIDout = 0.0
                        self.lwheel, self.rwheel = controller.homing_multiplier(PIDout)
                        self.counter += 1
                    # speed = 0.08
                else:
                    pixel_error = x["error"]
                    self.stop = x["stop"]
                    self.logger.info(f"self.stop = {self.stop} and type(self.stop) = {type(self.stop)}")
                    self.logger.info("ball detected, control initiated...")
                    self.counter = 0
                    # run PID with ball position error to make an adjustment
                    PIDout = controller.PID(float(pixel_error))
                    self.lwheel, self.rwheel = controller.homing_multiplier(PIDout)
                    # if -20 <= PIDout <= 20:
                    #     speed = 0.08
                    # else: 
                    #     speed = 0.05
                # drive with control if we got a package from pc
                if PIDout is None:
                    self.logger.error("error, didn't fetch from pc and skipped ahead")
                elif self.stop=='True':
                    self.logger.info("stop commanded, arrived at ball")
                    #drive backwards, retrace our steps
                    robot.set_1D_direction(dirForward=True)
                    for index in range(-1, -len(self.stored_pathway) -1, -1):
                        element = self.stored_pathway[index]
                    # run through all the steps we took backwards, setting our left wheel as right and right as left.
                        robot.drive(distance=0.2, speed=0.1, leftwheel_multilpier=element[0], rightwheel_multiplier=element[1])
                        self.logger.info(f'backwards: left wheel {round(element[0],2)} right wheel {round(element[1],2)}')
                    robot.pi.hardware_PWM(robot.stepL, 0, 500000)
                    robot.pi.hardware_PWM(robot.stepR, 0, 500000)
                    break
                else:
                    # save the multiplier for bth wheels together, used for reversing
                    both_wheelsLR = [self.lwheel, self.rwheel]
                    self.stored_pathway.append(both_wheelsLR)
                    #print("stored path so far:", self.stored_pathway)
                    self.logger.info(f'forwards: left wheel {round(self.lwheel,2)} right wheel {round(self.rwheel,2)}')
                    # drive the way commanded by wheel values
                    robot.set_1D_direction(dirForward=False)
                    robot.drive(distance=0.2, speed=0.1, leftwheel_multilpier=self.lwheel, rightwheel_multiplier=self.rwheel) 
                
            except: # catch all exceptions including KeyboardInterrupt
                # terminate gracefully
                robot.all_stop()
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
