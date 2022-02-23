import RPi.GPIO as GPIO
import threading
import time

# set gpio pin mode
GPIO.setmode(GPIO.BCM)

# thruster gpio pins
thrusters = [17, 27, 22, 10, 9, 11, 0, 5]

# setup thruster gpio pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(thrusters, GPIO.OUT, initial=GPIO.LOW)


def thruster_control(u):

    def fire_thruster(pin, name, u):
            dt = u if u > 0.01 else 0
            
            GPIO.output(pin, GPIO.HIGH)
            print(name, 'ON')

            time.sleep(dt)

            GPIO.output(pin, GPIO.LOW)
            print(name, 'OFF')

    # create a list of threads
    thruster_threads_list = []

    # initialize and start threads - takes 3-4ms to do
    for i in range(8):
        t = threading.Thread(target=fire_thruster, 
                             name="thruster{}".format(i+1), 
                             args=(thrusters[i],"thruster{}".format(i),u[i]))
        thruster_threads_list.append(t)     # append threads into list
        t.start()   # start thread
        print(t.name, "has started")

    # continue control loop after every thruster fired
    for t in thruster_threads_list:
        t.join()




