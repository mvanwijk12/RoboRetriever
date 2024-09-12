# practice for turning by bec

import time
from drive import Drive
from Pcontrol import Controller

if __name__ == "__main__":
    d = 0.3
    while True:
        robot = Drive()
    
        try:
            robot.set_1D_direction(dirForward=False)
            robot.drive(distance=d, speed=0.1, leftwheel_multilpier=1, rightwheel_multiplier=1)
            robot.drive(distance=d, speed=0.1, leftwheel_multilpier=1.5, rightwheel_multiplier=1) 
            robot.drive(distance=d, speed=0.1, leftwheel_multilpier=1, rightwheel_multiplier=1.5)
            robot.drive(distance=d, speed=0.1, leftwheel_multilpier=1, rightwheel_multiplier=1)

            robot.set_1D_direction(dirForward=True)
            robot.drive(distance=d, speed=0.1, leftwheel_multilpier=1, rightwheel_multiplier=1)
            robot.drive(distance=d, speed=0.1, leftwheel_multilpier=1, rightwheel_multiplier=1.5) 
            robot.drive(distance=d, speed=0.1, leftwheel_multilpier=1.5, rightwheel_multiplier=1)
            robot.drive(distance=d, speed=0.1, leftwheel_multilpier=1, rightwheel_multiplier=1)
        except KeyboardInterrupt:
            # terminate gracefully
            robot.all_stop()
            GPIO.cleanup()
            sys.exit()


# if __name__ == "__main__":
#     while True:
#         robot = Drive()
        
#         try:
#             robot.tank_turn(direction=1, amount=0.1, speed=0.05)
#             robot.all_stop()
#             start_time = time.time()
#             while time.time() - start_time < 3:
#                 pass
#         except KeyboardInterrupt:
#             # terminate gracefully
#             robot.all_stop()
#             GPIO.cleanup()
#             sys.exit()