import RPi.GPIO as GPIO
import time

# thruster gpio pins
thrusters = [17, 27, 22, 10, 9, 11, 0, 5]

# setup thruster gpio pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(thrusters, GPIO.OUT, initial=GPIO.LOW)

GPIO.output(9, GPIO.HIGH)
GPIO.output(27, GPIO.HIGH)
print("hi")
time.sleep(1)
print("after sleep")
GPIO.output(9, GPIO.LOW)
GPIO.output(27, GPIO.LOW)
print("low")


GPIO.cleanup()

# try:
#     while True:
#         GPIO.output(pin1, GPIO.HIGH)
#         print('Relay 1 ON')
#         time.sleep(2)

#         GPIO.output(pin1, GPIO.LOW)
#         print('Relay 1 OFF')
#         time.sleep(2)

# except KeyboardInterrupt:   #ctrl-c
#     print("Cleaning up GPIO")
# finally:
#     GPIO.cleanup()

