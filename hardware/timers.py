# timer_setup.py

import time
import threading
from datetime import datetime, timedelta

def timer_function(name):
    """Function to be executed when the timer expires."""
    print(f"{name} timer expired at {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")

def setup_timer(name, duration_seconds, function):
    """Sets up a timer to execute a function after a certain duration."""
    def timer_thread():
        print(f"{name} timer started at {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        time.sleep(duration_seconds)
        function(name)

    thread = threading.Thread(target=timer_thread)
    thread.start()

def main():
    # Set up multiple timers as needed
    setup_timer("Timer 1", 5, timer_function)  # 5 seconds

    # This is to keep the main thread alive to allow other threads to run
    while threading.active_count() > 1:
        time.sleep(1)

if __name__ == "__main__":
    main()
