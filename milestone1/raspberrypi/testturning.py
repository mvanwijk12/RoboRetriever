# practice for turning by bec

import time
from drive import Drive
from Pcontrol import Controller


robot = Drive()
robot.set_1D_direction(dirForward=False)
robot.drive(distance=1, speed=0.1, leftwheel_multilpier=1, rightwheel_multiplier=1)
robot.drive(distance=1, speed=0.1, leftwheel_multilpier=1.3, rightwheel_multiplier=1) 
robot.drive(distance=1, speed=0.1, leftwheel_multilpier=1, rightwheel_multiplier=1)
robot.drive(distance=1, speed=0.1, leftwheel_multilpier=1, rightwheel_multiplier=1.3)

robot.set_1D_direction(dirForward=True)
robot.drive(distance=1, speed=0.1, leftwheel_multilpier=1, rightwheel_multiplier=1)
robot.drive(distance=1, speed=0.1, leftwheel_multilpier=1.3, rightwheel_multiplier=1) 
robot.drive(distance=1, speed=0.1, leftwheel_multilpier=1, rightwheel_multiplier=1)
robot.drive(distance=1, speed=0.1, leftwheel_multilpier=1, rightwheel_multiplier=1.3)