import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)


#relay 1
GPIO.setup(2, GPIO.OUT)
#relay 2
GPIO.setup(3, GPIO.OUT)

try:
    while True:
        GPIO.output(2, GPIO.HIGH)
        print('Relay 1 ON')
        time.sleep(2)
        GPIO.output(3, GPIO.HIGH)
        print('Relay 2 ON')
        time.sleep(2)
        GPIO.output(2, GPIO.LOW)
        print('Relay 1 OFF')
        time.sleep(2)
        GPIO.output(3, GPIO.LOW)
        print('Relay 2 OFF')
        time.sleep(2)
finally:
    GPIO.cleanup()

