# This is the main code
#import pigpio
import time
#import RPi.GPIO as GPIO
from drive import Drive
from Pcontrol import Controller
from server import ConnectionServer

# Constants
LIMIT_SWITCH_PIN = 10
TIMEOUT = 30

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
        # self.start_time = None
      

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
                    print("no ball detected, driving straight")
                    robot.set_1D_direction(dirForward=False)
                    PIDout = 0.0
                    # speed = 0.08
                else:
                    pixel_error = x["error"]
                    self.stop = x["stop"]
                    print(f"self.stop = {self.stop} and type(self.stop) = {type(self.stop)}")
                    print("ball detected, control initiated...")
                    # run PID with ball position error to make an adjustment
                    PIDout = controller.PID(float(pixel_error))
                    # if -20 <= PIDout <= 20:
                    #     speed = 0.08
                    # else: 
                    #     speed = 0.05
                # drive with control if we got a package from pc
                if PIDout is None:
                    print("error, didn't fetch from pc and skipped ahead")
                elif self.stop=='True':
                    print("stop commanded, arrived at ball")
                    #drive backwards, retrace our steps
                    robot.set_1D_direction(dirForward=True)
                    for current_actionLR in self.stored_pathway:
                    # run through all the steps we took backwards, setting our left wheel as right and right as left.
                        robot.drive(distance=0.2, speed=0.1, leftwheel_multilpier=current_actionLR[0], rightwheel_multiplier=current_actionLR[1])
                        print('backwards: left wheel ', round(current_actionLR[0],2), ', right wheel ', round(current_actionLR[1],2))
                    robot.pi.hardware_PWM(robot.stepL, 0, 500000)
                    robot.pi.hardware_PWM(robot.stepR, 0, 500000)
                    break
                else:
                    self.lwheel, self.rwheel = controller.homing_multiplier(PIDout)
                    # save the multiplier for bth wheels together, used for reversing
                    both_wheelsLR = [self.lwheel, self.rwheel]
                    self.stored_pathway.append(both_wheelsLR)
                    #print("stored path so far:", self.stored_pathway)
                    print('forwards: left wheel ', round(self.lwheel,2), ', right wheel ', round(self.rwheel,2))
                    robot.set_1D_direction(dirForward=False)
                    robot.drive(distance=0.2, speed=0.1, leftwheel_multilpier=self.lwheel, rightwheel_multiplier=self.rwheel) 
                
            except KeyboardInterrupt:
            # terminate gracefully
                robot.pi.hardware_PWM(robot.stepL, 0, 500000)
                robot.pi.hardware_PWM(robot.stepR, 0, 500000)
                #GPIO.cleanup()
                #sys.exit()


    def run(self):
        try:
            self.main_loop()
        finally:
            print('system ended')
            # self.cleanup()

if __name__ == "__main__":
    robot_controller = RobotController()
    robot_controller.run()
