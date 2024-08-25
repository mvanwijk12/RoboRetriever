# This is the main code
#import pigpio
import time
#import RPi.GPIO as GPIO
from drive import Drive
from Pcontrol import Controller
from server import ConnectionServer

# Constants
LIMIT_SWITCH_PIN = 10
TIMEOUT = 40

class RobotController:
    def __init__(self):
        self.stored_pathway = []
        self.lwheel_sum = 0
        self.rwheel_sum = 0
        self.angle_line = None
        self.start_time = time.time()
        self.lwheel = 0.0
        self.rwheel = 0.0

        # Set up GPIO mode
        #GPIO.setmode(GPIO.BCM)
        
        # Set up the GPIO pin as an input
        #GPIO.setup(LIMIT_SWITCH_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # Initialize pigpio
        #self.pi = pigpio.pi()

    def setup_connection(self):
        con = ConnectionServer().start()
        return con

    def main_loop(self):
        robot = Drive()
        # input to controller includes the gains kp, ki, kd. these can be adjusted for tuning
        controller = Controller(0.0008,0,0)
        pixel_error = 0.0
        PIDout = None
        while True:
            # Fetch data from laptop
            con = self.setup_connection()
            try:
                x = con.get_message()
                if x is None:
                    print("no ball detected, driving straight")
                    robot.set_1D_direction(dirForward=False)
                    PIDout = 0.0
                else:
                    pixel_error = x["error"]
                    print("ball detected, control initiated...")
                    # run PID with ball position error to make an adjustment
                    PIDout = controller.PID(float(pixel_error))
            
            self.lwheel, self.rwheel = controller.homing_multiplier(PIDout)
            # save the multiplier for bth wheels together, used for reversing
            both_wheelsLR = [self.lwheel, self.rwheel]
            self.stored_pathway.append(both_wheelsLR)
            print("stored path so far:", self.stored_pathway)
            # # alternative
            # lwheel_sum += lwheel
            # rwheel_sum += rwheel
            print('left wheel ', round(lwheel,2), ', right wheel ', round(rwheel,2))
            
            robot.set_1D_direction(dirForward=False)
            robot.drive(distance=0.2, speed=0.2, leftwheel_multilpier=lwheel, rightwheel_multiplier=rwheel)
            #drive slowly slow down 
                
            except KeyboardInterrupt:
            # terminate gracefully
                robot.pi.hardware_PWM(robot.stepL, 0, 500000)
                robot.pi.hardware_PWM(robot.stepR, 0, 500000)
                GPIO.cleanup()
                sys.exit()

            

    def cleanup(self):
        robot.pi.hardware_PWM(robot.stepL, 0, 500000)
        robot.pi.hardware_PWM(robot.stepR, 0, 500000)
        GPIO.cleanup()

    def run(self):
        try:
            self.main_loop()
        finally:
            self.cleanup()

if __name__ == "__main__":
    robot_controller = RobotController()
    robot_controller.run()