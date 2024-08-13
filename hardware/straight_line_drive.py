#!/usr/bin/python

import pigpio
import os
import sys
from time import sleep
import time
import threading
from datetime import datetime, timedelta
import RPi.GPIO as GPIO

def timer_function(name):
    """Function to be executed when the timer expires."""
    pi.hardware_PWM(step_pin_R, 0, 500000)
    pi.hardware_PWM(step_pin_L, 0, 500000)

    print(f"{name} timer expired at {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")

def setup_timer(name, duration_seconds, function):
    """Sets up a timer to execute a function after a certain duration."""
    def timer_thread():
        print(f"{name} timer started at {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        sleep(duration_seconds)
        function(name)

    thread = threading.Thread(target=timer_thread)
    thread.start()

def main(forward=True, pps=2000, time=1):

    # Set up multiple timers as needed
    setup_timer("Timer 1", time, timer_function)  # 1 seconds
    pi.hardware_PWM(step_pin_R, pps, 500000)
    pi.hardware_PWM(step_pin_L, pps, 500000)

    # This is to keep the main thread alive to allow other threads to run
    while threading.active_count() > 1:
        sleep(1)

if __name__ == "__main__":
    try:
        os.system("sudo pigpiod")
        pi = pigpio.pi()

        # Define pins
        step_pin_L = 18
        step_pin_R = 19

        dir_pin_L = 24
        dir_pin_R = 23

        # Set direction
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(dir_pin_L, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(dir_pin_R, GPIO.OUT, initial=GPIO.HIGH)
        GPIO.output(dir_pin_L, GPIO.LOW)
        GPIO.output(dir_pin_R, GPIO.HIGH)

        main(time=3)

    except KeyboardInterrupt:
        # terminate gracefully
        pi.hardware_PWM(step_pin_R, 0, 500000)
        pi.hardware_PWM(step_pin_L, 0, 500000)
        GPIO.cleanup()
        sys.exit()
