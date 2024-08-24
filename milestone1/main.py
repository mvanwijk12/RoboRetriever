# This is the main code
import cv2
import numpy as np
from ultralytics import YOLO
import time
from drive import Drive
from Pcontrol import Controller

# set up storage for our path to allow reverse to drive home
stored_pathway = []
# alternative storage
lwheel_sum = 0
rwheel_sum = 0
angle_line = None

# Loop code
while True:
    ### fetch from laptop: are there any lines in danger zone?
    # output from linedetect is an angle 
    # if yes turn away from line
    if angle_line is not None:
        if 0 <= angle_line <= 180:
            # line is in the right hand side of frame. so turn LEFT
            lwheel = 0.5
            rwheel = 1.5
            both_wheelsLR = [lwheel, rwheel]
            stored_pathway.append(both_wheelsLR)
            # alternative
            lwheel_sum += lwheel
            rwheel_sum += rwheel
            print('left wheel ', round(lwheel,2), ', right wheel ', round(rwheel,2))
            robot = Drive()
            robot.set_1D_direction(dirForward=True)
            robot.drive(distance=0.2, speed=0.2, leftwheel_multilpier=lwheel, rightwheel_multiplier=rwheel)
        else:
            # line is in the left hand side of frame. so turn RIGHT
            lwheel = 1.5
            rwheel = 0.5
            both_wheelsLR = [lwheel, rwheel]
            stored_pathway.append(both_wheelsLR)
            # alternative
            lwheel_sum += lwheel
            rwheel_sum += rwheel
            print('left wheel ', round(lwheel,2), ', right wheel ', round(rwheel,2))
            robot = Drive()
            robot.set_1D_direction(dirForward=True)
            robot.drive(distance=0.2, speed=0.2, leftwheel_multilpier=lwheel, rightwheel_multiplier=rwheel)
    

    ### Detect when reached target (ball is too close to bottom of frame)
    # get from network = targetreached = true
    if target_reached == True:
        # drive backwards, retrace our steps
        robot = Drive()
        robot.set_1D_direction(dirForward=False)
        for current_actionLR in stored_pathway:
            # run through all the steps we took backwards, setting our left wheel as right and right as left.
            robot.drive(speed=0.2, leftwheel_multilpier=current_actionLR[1], rightwheel_multiplier=current_actionLR[0])
    if target_reached == True:
        # drive backwards by the summed proportional amount of each wheel
        robot.Drive()
        robot.set_1D_direction(dirForward=False)
         = 
    else:
        ## fetch error of ball position from the laptop
        error = () # output of inference file

        ### run the pcontrol for one movement
        # input to controller includes the gains kp, ki, kd. these can be adjusted for tuning
        controller = Controller(0.0008,0,0)
        # running PID uses the ball position error to make an adjustment
        PIDout = controller.PID(int(error))
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
        robot.set_1D_direction(dirForward=True)
        robot.drive(speed=0.2, leftwheel_multilpier=lwheel, rightwheel_multiplier=rwheel)
            