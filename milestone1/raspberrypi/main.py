# This is the main code
import pigpio
import time
import RPi.GPIO as GPIO
from drive import Drive
from Pcontrol import Controller
#from server import ConnectionServer

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
        GPIO.setmode(GPIO.BCM)
        
        # Set up the GPIO pin as an input
        GPIO.setup(LIMIT_SWITCH_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # Initialize pigpio
        self.pi = pigpio.pi()

    def setup_connection(self):
        con = ConnectionServer().start()
        return con

    def main_loop(self):
        robot = Drive()
        # input to controller includes the gains kp, ki, kd. these can be adjusted for tuning
        controller = Controller(0.0008,0,0)
        pixel_error = 0.0
        PIDout = 0.0
        while True:
            # Fetch data from laptop
            con = self.setup_connection()
            try:
                x = con.get_message()
                if x is None:
                    print("no ball detected, drive straight")
                    robot.set_1D_direction(dirForward=False)
                    # robot.drive(distance=1, speed=0.2)
                    PIDout = 0.0
                else:
                    pixel_error = x["error"]
                    print("ball detected, control initiated...")
                    # run PID with ball position error to make an adjustment
                    PIDout = controller.PID(float(pixel_error))
            except:
                print("Error: Coild not get message from server")
            # homing takes output of PID 
            lwheel, rwheel = controller.homing_multiplier(PIDout)
            # save the multiplier for bth wheels together, used for reversing
            both_wheelsLR = [lwheel, rwheel]
            stored_pathway.append(both_wheelsLR)
            # alternative
            lwheel_sum += lwheel
            rwheel_sum += rwheel
            print('left wheel ', round(lwheel,2), ', right wheel ', round(rwheel,2))
            robot = Drive()
            robot.set_1D_direction(dirForward=False)
            robot.drive(speed=0.2, leftwheel_multilpier=lwheel, rightwheel_multiplier=rwheel)

            

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


#     ## fetch from laptop: are there any lines in danger zone?
#     output from linedetect is an angle 
#     if yes turn away from line
#     if angle_line is not None:
#         if 0 <= angle_line <= 180:
#             # line is in the right hand side of frame. so turn LEFT
#             lwheel = 0.5
#             rwheel = 1.5
#             both_wheelsLR = [lwheel, rwheel]
#             stored_pathway.append(both_wheelsLR)
#             # alternative
#             lwheel_sum += lwheel
#             rwheel_sum += rwheel
#             print('left wheel ', round(lwheel,2), ', right wheel ', round(rwheel,2))
#             robot = Drive()
#             robot.set_1D_direction(dirForward=False)
#             robot.drive(distance=0.2, speed=0.2, leftwheel_multilpier=lwheel, rightwheel_multiplier=rwheel)
#         else:
#             # line is in the left hand side of frame. so turn RIGHT
#             lwheel = 1.5
#             rwheel = 0.5
#             both_wheelsLR = [lwheel, rwheel]
#             stored_pathway.append(both_wheelsLR)
#             # alternative
#             lwheel_sum += lwheel
#             rwheel_sum += rwheel
#             print('left wheel ', round(lwheel,2), ', right wheel ', round(rwheel,2))
#             robot = Drive()
#             robot.set_1D_direction(dirForward=False)
#             robot.drive(distance=0.2, speed=0.2, leftwheel_multilpier=lwheel, rightwheel_multiplier=rwheel)
    

#     # Detect when reached target (ball hits limit switch)
#     #get from limit switch = targetreached = true
#     if GPIO.input(LIMIT_SWITCH_PIN) == False:

#     if time.time() - start_time >= timeout:
#         break
#         drive backwards, retrace our steps
#         robot = Drive()
#         robot.set_1D_direction(dirForward=True)
#         for current_actionLR in stored_pathway:
#             # run through all the steps we took backwards, setting our left wheel as right and right as left.
#             robot.drive(speed=0.1, leftwheel_multilpier=current_actionLR[1], rightwheel_multiplier=current_actionLR[0])
#         break
#     # fetch error of ball position from the laptop
    
#     try:
#         print(f'MESSAGE IS {con.get_message()}')
#         x = con.get_message()
#         if x is not None:
#             error = x["error"]
#     except KeyboardInterrupt:
#         con.close()
#         print('closing..')

#     if x is None:
#         print("no ball detected, drive straight")
#         robot = Drive()
#         robot.set_1D_direction(dirForward=False)
#         robot.drive(distance=1, speed=0.2)
#     else:
#         print("ball detected, control initiated...")
#         ### run the pcontrol for one movement
#         # input to controller includes the gains kp, ki, kd. these can be adjusted for tuning
#         controller = Controller(0.0008,0,0)
#         # running PID uses the ball position error to make an adjustment
#         PIDout = controller.PID(float(error))
#         # homing takes output of PID 
#         lwheel, rwheel = controller.homing_multiplier(PIDout)
#         # save the multiplier for bth wheels together, used for reversing
#         both_wheelsLR = [lwheel, rwheel]
#         stored_pathway.append(both_wheelsLR)
#         # alternative
#         lwheel_sum += lwheel
#         rwheel_sum += rwheel
#         print('left wheel ', round(lwheel,2), ', right wheel ', round(rwheel,2))
#         robot = Drive()
#         robot.set_1D_direction(dirForward=False)
#         robot.drive(speed=0.2, leftwheel_multilpier=lwheel, rightwheel_multiplier=rwheel)

# KeyboardInterrupt:
#     # terminate gracefully
#     robot.pi.hardware_PWM(robot.stepL, 0, 500000)
#     robot.pi.hardware_PWM(robot.stepR, 0, 500000)
#     GPIO.cleanup()
#     sys.exit()




    
#     # or the other wway of doing this
#     if GPIO.input(LIMIT_SWITCH_PIN) == False:
#         # drive backwards by the summed proportional amount of each wheel
#         robot.Drive()
#         robot.set_1D_direction(dirForward=False)
#         lwheel_norm = lwheel_sum/rwheel_sum
#         rwheel_norm = 1
#         # drive turning by proportional total amount
#         robot.drive(speed=0.2, leftwheel_multilpier=rwheel_norm, rightwheel_multiplier=lwheel_norm)
#         travelled_distance = int(stored_pathway.length/2)
#         # drive straight for the distance travelled.
#         robot.drive(distance=travelled_distance,speed=0.2, leftwheel_multilpier=0.5, rightwheel_multiplier=0.5)

