# This is the main code
import cv2
import numpy as np
from ultralytics import YOLO
import time
from drive import Drive
from Pcontrol import Controller

# set up storage for our path to allow reverse to drive home
stored_pathway = []

# Loop code
while True:
    ### fetch from laptop: are there any lines in danger zone?
    # output from linedetect is an angle 
    # if yes turn away from line
    if(0<=angle_line<=180):
        # line is in the right hand side of frame. so turn left
        both_wheelsLR = [0.5, 1.5]
        stored_pathway.append(both_wheelsLR)
        print('left wheel ', round(lwheel,2), ', right wheel ', round(rwheel,2))
        robot = Drive()
        robot.set_1D_direction(dirForward=True)
        robot.drive(speed=0.2, leftwheel_multilpier=lwheel, rightwheel_multiplier=rwheel)


    ### Detect when reached target (ball is too close to bottom of frame)
    # get from network = targetreached = true
    if(targetReached=True):
        # drive backwards
        robot = Drive()
        robot.set_1D_direction(dirForward=False)
        for current_actionLR in length.stored_pathway:
            # 
            robot.drive(speed=0.2, leftwheel_multilpier=current_actionLR[1], rightwheel_multiplier=current_actionLR[0])
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
        print('left wheel ', round(lwheel,2), ', right wheel ', round(rwheel,2))
        robot = Drive()
        robot.set_1D_direction(dirForward=True)
        robot.drive(speed=0.2, leftwheel_multilpier=lwheel, rightwheel_multiplier=rwheel)
        
    
            