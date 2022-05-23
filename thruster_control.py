import RPi.GPIO as GPIO
import threading
import time
import numpy as np


# thruster gpio pins
thrusters = [17, 27, 22, 10, 9, 11, 0, 5]

# setup thruster gpio pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(thrusters, GPIO.OUT, initial=GPIO.HIGH)  # relays OFF

def fire_thruster(pin, name, fireTimes):
    dt = fireTimes if fireTimes > 0.01 else 0

    GPIO.output(pin, GPIO.LOW)
    print(name, 'ON')
    time.sleep(dt)
    GPIO.output(pin, GPIO.HIGH)
    print(name, 'OFF')

def thruster_control(fireTimes):

    # create a list of threads
    thruster_threads_list = []

    # initialize and start threads - takes 3-4ms to do
    for i in range(8):
        t = threading.Thread(target=fire_thruster, 
                             name="thruster{}".format(i+1), 
                             args=(thrusters[i],"thruster{}".format(i+1),fireTimes[i][0]))
        thruster_threads_list.append(t)     # append threads into list
        t.start()   # start thread
        print(t.name, "has started")

    # continue control loop after every thruster fired
    for t in thruster_threads_list:
        t.join()



# testing
# fireTimes = [0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05]
# fireTimes = [1, 2, 3, 4, 5, 6, 7, 8]
fireTimes = np.array([[1], [1], [1], [1], [1], [1], [1], [1]])     # pose from gps and imu

thruster_control(fireTimes)

print("all thrusters fired")

# GPIO.cleanup()

