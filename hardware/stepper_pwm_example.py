#!/usr/bin/python

import pigpio
import os
import sys
from time import sleep

os.system ("sudo pigpiod")
pi = pigpio.pi()
freq = 0

try:
    for i in range(61):
        # set hardware pwm on GPIO19 with 50% duty cycle
        pi.hardware_PWM(19, freq, 500000)
        print(f'step frequency: {freq}Hz')
        freq += 50
        sleep(1)
    
    # turn pwm off
    pi.hardware_PWM(19, 0, 0)

except KeyboardInterrupt:
    # terminate gracefully
    pi.hardware_PWM(19, 0, 0)
    sys.exit()
