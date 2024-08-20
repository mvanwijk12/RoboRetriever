# this contains the names of each state and controls how we pass between each state. 
# made by Bec 20/08/2024

class BallDetect:
    def on_event(self, event):
        if event == "trigger_pcontrol":
            return PControlState()
        elif event == "arrived_at_ball":
            return ReturnHome()
        elif event == "no_ball_detected":
            return RotateState()
        else:
            return self

class RotateState:
    def on_event(self, event):
        if event == "complete":
            return BallDetect()
        else:
            return self


class PControlState:
    controller = Controller(0.0008,0,0)
    # provide to the PID function the error from the centreline. so "pixel_from_left" - centreline
    error = input("enter error:")
    out = controller.PID(int(error))
    lwheel, rwheel = controller.homing_multiplier(out)
    print('left wheel ', round(lwheel,2), ', right wheel ', round(rwheel,2))
    robot = Drive()
    robot.set_1D_direction(dirForward=False)
    robot.drive(speed=0.2, leftwheel_multilpier=lwheel, rightwheel_multiplier=rwheel)


    def on_event(self, event):
        if event == "completed":
            return BallDetect()
        else:
            return self

class ReturnHome:
    def on_event(self, event):
        return self 
        # final state


# define statemachine
class StateMachine:
    def __init__(self):
        self.state = BallDetect()

    def on_event(self, event):
        self.state = self.state.on_event(event)
        
# run this only when this file is run
if __name__ == "__main__":
    state_machine = StateMachine()
