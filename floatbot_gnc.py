from marvelmind import MarvelmindHedge
from mpu6050 import mpu6050
import RPi.GPIO as GPIO
import numpy as np
import threading
import time

""" set up imu gyro """
mpu = mpu6050(0x68)
mpu.set_gyro_range(0x00)
""" set up imu gyro """

""" GYRO Z-AXIS CALIBRATION """
print("Calibrating Gyro Yaw (z-axis)... ")
count = 0
gyroReadings = 420     # number of gyro data to pull
gyro_z_vals = np.zeros((1, gyroReadings))   # arrary to store gyro values  
while True:

    # pulling gyro data
    gyro_data = mpu.get_gyro_data()

    # store gyro values in an array
    gyro_z_vals[0, count] = gyro_data['z']

    # counter to end while loop
    count += 1

    # once pulled enough gyro readings
    if count == gyroReadings:
        gyroZBias = sum(gyro_z_vals) / np.size(gyro_z_vals) 
        zth = np.amax(gyro_z_vals)    # max bias in array
        print("Read", np.size(gyro_z_vals), "gyro readings")
        time.sleep(1)
        print("Avg gyro yaw bias (z-axis) is: ", gyroZBias)
        time.sleep(1)
        print("Largest absolute gyro rate val: ", zth)
        break
""" GYRO Z-AXIS CALIBRATION """

""" FLOATBOT NAVIGATION THREAD FUNCTION """
# inital conditions
yawAng = 0.0            # set initial yaw angle
prevYawAng = 0.0        
gyro_data = 0.0         # set initial gyro rate
gyroUpdateRate = 0.05   # 50ms == 20Hz update rate
dt = 0
prev_gryo_data = 0.0

# position and angle of floatbot
floatbot_pose = np.zeros(3)

def floatbot_navigation():
    
    global floatbot_pose

    #Define GPS Hedgehog object, need to check the port
    hedge = MarvelmindHedge(tty = "/dev/ttyACM1", adr = None, debug = False)
    print("Starting GPS...")
    hedge.start()
    
    while True:
        startTime = time()

        # get GPS position and parse
        pos = hedge.position()
        x = pos[1]
        y = pos[2]

        # compute angle at current timestep
        if prev_gryo_data == 0.0:
            yawAng = prevYawAng
            print(yawAng)
        else:
            yawAng = prevYawAng + (prev_gryo_data-gyroZBias)*gyroUpdateRate
            print(yawAng)

        # get gyro rate
        gyro_data = mpu.get_gyro_data()     # pull gyro z rate

        if -zth <= gyro_data['z']-gyroZBias <= zth:
            prev_gryo_data = 0.0
            prevYawAng = yawAng
            # print("ZERO BABY!!!")
        elif -zth < gyro_data['z']-gyroZBias > zth:
            prev_gryo_data = gyro_data['z']
            prevYawAng = yawAng
            # print("or elif")
        else:
            print("something else happened")

        # store pose in array
        floatbot_pose[0] = x
        floatbot_pose[1] = y
        floatbot_pose[2] = yawAng
        
        endTime = time()

        # dt = endTime - startTime  # OR FORGET THE TIMESTEP AND JUST USE THE TIME IT TAKES TO FINISH ONE LOOP

        h = gyroUpdateRate - (endTime - startTime)     # time taken out of timestep
        time.sleep(h)    # pause loop until next timestep

nav_thread = threading.Thread(target=floatbot_navigation, daemon = True)
nav_thread.start()
print("Started Floatbot navigation")
""" FLOATBOT NAVIGATION THREAD FUNCTION """

""" PID CONTROLLER """
# pid gains
kp = 1
ki = 1
kd = 1
intError = 0

def pid(ref, nav):    
    currentTime = time()                            # get current time
    dt = currentTime - previousTime                 # compute time elapsed from previous computation
    
    error = ref - nav                               # determine error
    intError += error * dt                # compute integral
    rateError = (error - lastError)/dt              # compute derivative

    u = kp*error + ki*intError + kd*rateError         # PID output               

    lastError = error                               # remember current error
    previousTime = currentTime                      # remember current time

    thruster_matrix = np.array([[-1, 1, 0, 0, 1, -1, 0, 0], 
                                [0, 0, 1, -1, 0, 0, -1, 1], 
                                [-1, 1, -1, 1, -1, 1, -1, 1]])

    p = np.linalg.pinv(thruster_matrix)

    return u                                        # function returns the PID output
""" PID CONTROLLER """




# NEED TO WRITE SOME USER INTERFACE CODE HERE
# reference point
ref = [10, 10, 0]


# control loop parameters
h = 0.2   # 5Hz control loop 

previousTime = time()   # initial previous time

try:
    while True:
        
        u = pid(floatbot_pose)
        # thruster control
        # sleep

except KeyboardInterrupt:
    GPIO.cleanup()
    print("CLEANED UP GPIO")
    time.sleep(1)
    print('TERMINATED CONTROL LOOP')