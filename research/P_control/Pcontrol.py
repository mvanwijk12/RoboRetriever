import numpy
import time

class Controller:

    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.integral = 0.0
        self.last_pixel = 0.0
        self.last_update = 0.0
        

    def PID(self, input_pixel):
        dt = time.time() - self.last_update

        p = input_pixel*self.kp

        self.integral += input_pixel*dt
        i = self.integral*self.ki

        derivative = (input_pixel - self.last_pixel)/dt
        d = derivative*self.kd
        self.last_pixel = input_pixel
        self.last_update = time.time()

        return p + i + d

    def homing_multiplier(self, inp):
        leftwheel_multilpier = numpy.tanh(inp)+1
        rightwheel_multiplier = numpy.tanh(-inp)+1

        return leftwheel_multilpier, rightwheel_multiplier



if __name__ == "__main__":
    controller = Controller(0.0002,0,0)
    # provide to the PID function the error from the centreline. so "pixel_from_left" - centreline
    out = controller.PID(-700)
    lwheel, rwheel = controller.homing_multiplier(out)
    print('left wheel ', round(lwheel,2), ', right wheel ', round(rwheel,2))
    # assuming we get a value between 0 and [width of image] say 600 for now.

  