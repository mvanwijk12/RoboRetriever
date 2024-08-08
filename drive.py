# sudo apt-get update
# sudo apt-get install python3-rpi.gpio

import RPi.GPIO as GPIO
import time

# Set up GPIO mode
GPIO.setmode(GPIO.BCM)

# Define GPIO pins
StepPins = [17, 18, 27, 22]

# Set all pins as output
for pin in StepPins:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, False)

# Define step sequence
StepSeq = [
    [1,0,0,1],
    [1,0,0,0],
    [1,1,0,0],
    [0,1,0,0],
    [0,1,1,0],
    [0,0,1,0],
    [0,0,1,1],
    [0,0,0,1]
]

StepCount = len(StepSeq)
StepDir = 1  # Set direction: 1 for clockwise, -1 for counterclockwise

WaitTime = 0.01  # Delay between steps

# Initialize step counter
StepCounter = 0

try:
    while True:
        for pin in range(4):
            xpin = StepPins[pin]
            if StepSeq[StepCounter][pin] != 0:
                GPIO.output(xpin, True)
            else:
                GPIO.output(xpin, False)

        StepCounter += StepDir

        # If at the end of the sequence, start again
        if (StepCounter >= StepCount):
            StepCounter = 0
        if (StepCounter < 0):
            StepCounter = StepCount + StepDir

        # Wait before moving to the next step
        time.sleep(WaitTime)

except KeyboardInterrupt:
    # Cleanup GPIO pins on exit
    GPIO.cleanup()



