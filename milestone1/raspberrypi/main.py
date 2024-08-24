# This is the main code
import pigpio
import time
import RPi.GPIO as GPIO
from drive import Drive
from Pcontrol import Controller
from server import ConnectionServer

####### fix limmit switch to be an interrupt so we don't accidentally miss the event of pressing it
# # Set up GPIO mode
# GPIO.setmode(GPIO.BCM)

# # Define GPIO pin for the limit switch
# LIMIT_SWITCH_PIN = 10 # set this

# # Set up the GPIO pin as an input
# GPIO.setup(LIMIT_SWITCH_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

con = ConnectionServer().start()

# set up storage for path to allow reverse to drive home
stored_pathway = []
# alternative storage
lwheel_sum = 0
rwheel_sum = 0
# position of line in image
angle_line = None

timeout = 20
start_time = int(time.time())

# test code fake values


# Loop code
while True:
    ### fetch from laptop: are there any lines in danger zone?
    # output from linedetect is an angle 
    # if yes turn away from line
    # if angle_line is not None:
    #     if 0 <= angle_line <= 180:
    #         # line is in the right hand side of frame. so turn LEFT
    #         lwheel = 0.5
    #         rwheel = 1.5
    #         both_wheelsLR = [lwheel, rwheel]
    #         stored_pathway.append(both_wheelsLR)
    #         # alternative
    #         lwheel_sum += lwheel
    #         rwheel_sum += rwheel
    #         print('left wheel ', round(lwheel,2), ', right wheel ', round(rwheel,2))
    #         robot = Drive()
    #         robot.set_1D_direction(dirForward=True)
    #         robot.drive(distance=0.2, speed=0.2, leftwheel_multilpier=lwheel, rightwheel_multiplier=rwheel)
    #     else:
    #         # line is in the left hand side of frame. so turn RIGHT
    #         lwheel = 1.5
    #         rwheel = 0.5
    #         both_wheelsLR = [lwheel, rwheel]
    #         stored_pathway.append(both_wheelsLR)
    #         # alternative
    #         lwheel_sum += lwheel
    #         rwheel_sum += rwheel
    #         print('left wheel ', round(lwheel,2), ', right wheel ', round(rwheel,2))
    #         robot = Drive()
    #         robot.set_1D_direction(dirForward=True)
    #         robot.drive(distance=0.2, speed=0.2, leftwheel_multilpier=lwheel, rightwheel_multiplier=rwheel)
    

    ## Detect when reached target (ball hits limit switch)
    ##get from limit switch = targetreached = true
    #if GPIO.input(LIMIT_SWITCH_PIN) == False:

    if time.time() - start_time >= timeout:
        # drive backwards, retrace our steps
        robot = Drive()
        robot.set_1D_direction(dirForward=True)
        for current_actionLR in stored_pathway:
            # run through all the steps we took backwards, setting our left wheel as right and right as left.
            robot.drive(speed=0.1, leftwheel_multilpier=current_actionLR[1], rightwheel_multiplier=current_actionLR[0])
        break
    # # or the other wway of doing this
    # if GPIO.input(LIMIT_SWITCH_PIN) == False:
    #     # drive backwards by the summed proportional amount of each wheel
    #     robot.Drive()
    #     robot.set_1D_direction(dirForward=False)
    #     lwheel_norm = lwheel_sum/rwheel_sum
    #     rwheel_norm = 1
    #     # drive turning by proportional total amount
    #     robot.drive(speed=0.2, leftwheel_multilpier=rwheel_norm, rightwheel_multiplier=lwheel_norm)
    #     travelled_distance = int(stored_pathway.length/2)
    #     # drive straight for the distance travelled.
    #     robot.drive(distance=travelled_distance,speed=0.2, leftwheel_multilpier=0.5, rightwheel_multiplier=0.5)


    ## fetch error of ball position from the laptop
    
    try:
        print(f'MESSAGE IS {con.get_message()}')
        x = con.get_message()
        if x is not None:
            error = x["error"]
    except KeyboardInterrupt:
        con.close()
        print('closing..')

    if x is None:
        robot = Drive()
        robot.set_1D_direction(dirForward=False)
        robot.drive(distance=1, speed=0.1)
    else:
        ### run the pcontrol for one movement
        # input to controller includes the gains kp, ki, kd. these can be adjusted for tuning
        controller = Controller(0.0008,0,0)
        # running PID uses the ball position error to make an adjustment
        PIDout = controller.PID(float(error))
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
        robot.drive(speed=0.1, leftwheel_multilpier=lwheel, rightwheel_multiplier=rwheel)
