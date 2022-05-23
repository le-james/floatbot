import RPi.GPIO as GPIO
import time

# thruster gpio pins
thrusters = [17, 27, 22, 10, 9, 11, 0, 5]

# setup thruster gpio pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(thrusters, GPIO.OUT, initial=GPIO.HIGH)

for i in thrusters:
    time.sleep(1)
    GPIO.output(i, GPIO.LOW)
    time.sleep(1)
    GPIO.output(i, GPIO.HIGH)



# time.sleep(1)
# GPIO.output(thrusters[0], GPIO.LOW)
# time.sleep(1)
# GPIO.output(thrusters[0], GPIO.HIGH)
# time.sleep(1)
# GPIO.output(thrusters[1], GPIO.LOW)
# time.sleep(1)
# GPIO.output(thrusters[1], GPIO.HIGH)
# time.sleep(1)
# GPIO.output(thrusters[2], GPIO.LOW)
# time.sleep(1)
# GPIO.output(thrusters[2], GPIO.HIGH)
# time.sleep(1)
# GPIO.output(thrusters[3], GPIO.LOW)
# time.sleep(1)
# GPIO.output(thrusters[3], GPIO.HIGH)
# time.sleep(1)
# GPIO.output(thrusters[4], GPIO.LOW)
# time.sleep(1)
# GPIO.output(thrusters[4], GPIO.HIGH)
# time.sleep(1)
# GPIO.output(thrusters[5], GPIO.LOW)
# time.sleep(1)
# GPIO.output(thrusters[5], GPIO.HIGH)
# time.sleep(1)
# GPIO.output(thrusters[6], GPIO.LOW)
# time.sleep(1)
# GPIO.output(thrusters[6], GPIO.HIGH)
# time.sleep(1)
# GPIO.output(thrusters[7], GPIO.LOW)
# time.sleep(1)
# GPIO.output(thrusters[7], GPIO.HIGH)
# time.sleep(1)



# print("firing thruster")
# GPIO.output(thrusters[0], GPIO.LOW)
# GPIO.output(thrusters[2], GPIO.LOW)
# GPIO.output(thrusters[4], GPIO.LOW)
# GPIO.output(thrusters[6], GPIO.LOW)
# time.sleep(0.25)
# GPIO.output(thrusters[0], GPIO.HIGH)
# GPIO.output(thrusters[2], GPIO.HIGH)
# GPIO.output(thrusters[4], GPIO.HIGH)
# GPIO.output(thrusters[6], GPIO.HIGH)

# time.sleep(5)

# print("reverse dirn")
# GPIO.output(thrusters[1], GPIO.LOW)
# GPIO.output(thrusters[3], GPIO.LOW)
# GPIO.output(thrusters[5], GPIO.LOW)
# GPIO.output(thrusters[7], GPIO.LOW)
# time.sleep(0.25)
# GPIO.output(thrusters[1], GPIO.HIGH)
# GPIO.output(thrusters[3], GPIO.HIGH)
# GPIO.output(thrusters[5], GPIO.HIGH)
# GPIO.output(thrusters[7], GPIO.HIGH)

GPIO.cleanup()