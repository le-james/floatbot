import time
import RPi.GPIO as GPIO

BUTTON_GPIO = 16
GPIO.setmode(GPIO.BCM)

# when this callback function is called it takes an input of channel(GPIO number)
def button_released_callback(channel):
    print("button just released")

GPIO.setup(BUTTON_GPIO, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# detect rising, bounce time is milliseconds
GPIO.add_event_detect(BUTTON_GPIO, GPIO.RISING, callback=button_released_callback, bounceTime=50)

try:
    while True:
        print("hello wrld")
        time.sleep(1)
except KeyboardInterrupt:
    GPIO.cleanup()
    print('exited while loop')