import pigpio
import time
import threading

# Constants
TACH_PIN = 17  # GPIO pin
LOG_INTERVAL = 1 # in seconds
PULSES_PER_REV = 2

def tachometer_callback(gpio, level, tick):
    """ Callback function to count tachometer pulses """
    global pulse_count
    pulse_count += 1

def calculate_rpm():
    """ Function to calculate fan RPM """
    global pulse_count, start_time

    while True:
        
        time.sleep(LOG_INTERVAL)  # RPM calculation interval
        elapsed_time = time.time() - start_time
        rpm = (pulse_count / PULSES_PER_REV) * (60 / elapsed_time) 
        print(f"RPM: {rpm:.2f}")
        
        # Reset for the next calculation
        pulse_count = 0
        start_time = time.time()


if __name__ == "__main__":

    # Global variables to track the number of pulses and the RPM
    pulse_count = 0
    start_time = time.time()

    # Setup pigpio
    pi = pigpio.pi()
    if not pi.connected:
        exit()

    # Set up tachometer pin as input
    pi.set_mode(TACH_PIN, pigpio.INPUT)
    pi.set_pull_up_down(TACH_PIN, pigpio.PUD_UP)

    # Set up interrupt for rising edges on the tach pin
    pi.callback(TACH_PIN, pigpio.RISING_EDGE, tachometer_callback)


    try:
        # Start a thread to calculate and log RPM
        rpm_thread = threading.Thread(target=calculate_rpm)
        rpm_thread.start()

        # Keep the program running
        while True:
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("Program stopped by User")

    finally:
        pi.stop()
