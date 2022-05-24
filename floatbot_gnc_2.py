from marvelmind import MarvelmindHedge
from mpu6050 import mpu6050
import RPi.GPIO as GPIO
from cmath import pi
import numpy as np
import threading
import time

""" set up imu gyro """
mpu = mpu6050(0x68)
mpu.set_gyro_range(0x00)
""" set up imu gyro """



""" GYRO Z-AXIS CALIBRATION CODE """
print("Calibrating Gyro Yaw (z-axis)... ")
count = 0
gyroReadings = 420     # number of gyro data to pull
sumZGyroReadings = 0
while True:
    # counter to end while loop
    count += 1

    # pulling gyro data
    gyro_data = mpu.get_gyro_data()

    # summming z gyro readings
    sumZGyroReadings += gyro_data['z']

    # once pulled enough gyro readings
    if count == gyroReadings:
        gyroZBias = sumZGyroReadings/count
        print("Read", gyroReadings, "gyro readings")
        time.sleep(1)
        print("Gyro yaw bias (z-axis) is: ", gyroZBias)
        break
""" GYRO Z-AXIS CALIBRATION CODE """



""" FLOATBOT NAVIGATION THREAD FUNCTION """

# position and angle of floatbot
floatbot_pose = np.zeros(3)

#Define GPS Hedgehog object, need to check the port
hedge = MarvelmindHedge(tty = "/dev/ttyACM0", adr = None, debug = False)
print("Starting GPS...")
hedge.start()
print("Started GPS!")

def floatbot_navigation():
    
    # stores nav states
    global floatbot_pose

    # gyro inital conditions
    yawAng = 0.0            # set initial yaw angle
    g5 = 0
    g4 = 0
    g3 = 0
    g2 = 0
    g1 = 0

    gyro_data = 0.0             # set initial gyro rate
    # gyroUpdateRate = 0.01     # 50ms == 20Hz update rate
    h = 0                       # gyro update rate
    thermalGain = 2             # the imu is affected by heat, it will start to read half the value
    prev_gryo_data = 0.0

    # previous gyro data

    while True:
        startTime = time.time()

        # get GPS position and parse
        pos = hedge.position()
        x = pos[1]
        y = pos[2]

        # compute angle at current timestep
        g5 = g4
        g4 = g3
        g3 = g2
        g2 = g1
        g1 = prev_gryo_data
        g = (g1+g2+g3+g4+g5)/5      # avg last 5 gyro readings
        yawAng += g*h*thermalGain

        # store prev gyro rate
        gyro_data = mpu.get_gyro_data()
        prev_gryo_data = gyro_data['z']-gyroZBias

        # store pose in array
        floatbot_pose[0] = x
        floatbot_pose[1] = y
        floatbot_pose[2] = yawAng
        # print("x: ", floatbot_pose[0], "y: ", floatbot_pose[1], "yaw: ", floatbot_pose[2])
        
        endTime = time.time()

        h = endTime - startTime + 0.05     # time taken out of timestep
        time.sleep(h)    # pause loop until next timestep

nav_thread = threading.Thread(target=floatbot_navigation, daemon = True)
nav_thread.start()
print("Starting Floatbot navigation...")
time.sleep(5)
print("Started Floatbot navigation!")
""" FLOATBOT NAVIGATION THREAD FUNCTION """



""" PID CONTROLLER """
# pid parameters
kpp = 0.75      # proportional postion gain
kpy = 0.01      # proportional yaw gain
ki = 1
kd = 1
intError = 0
lastError = 0
prevT = np.zeros(8)

# def pid(nav, ref):   
#     global intError, lastError, prevT

#     # pose error
#     error = nav - ref                      
#     error[2] = error[2]*pi/180

#     # distance vector
#     dis = np.array([error[0], error[1]])

#     if np.linalg.norm(dis) > 0.05 or np.abs(error[2]) > 5*pi/180:
#         # # pid error terms
#         # intError += error*dt                  # compute integral
#         # rateError = (error - lastError)/dt    # compute derivative

#         # pid output
#         u = kp*error #+ ki*intError + kd*rateError          

#         # # store previous error
#         # lastError = error
        
#         # location of each matric on floatbot
#         # from serc gnc paper
#         thruster_matrix = np.array([[-1, 1, 0, 0, 1, -1, 0, 0], 
#                                     [0, 0, 1, -1, 0, 0, -1, 1], 
#                                     [-1, 1, -1, 1, -1, 1, -1, 1]])
        
#         # psudoinverse of thruster matrix
#         mInv = np.linalg.pinv(thruster_matrix)
        
#         # time for each thruster
#         t = 2*np.matmul(mInv, u)#.round(decimals=2) # dont need to round since bounds takes care of it below

#         # thruster fire time bounds
#         t[t > 1] = 1
#         t[t < 0.1] = 0

#         # store prev t
#         prevT = t

#         # longest thruster time
#         dt = np.amax(t)
#         # flag = 1
#     elif np.linalg.norm(dis) < 0.05 and np.abs(error[2]) < 5*pi/180:
#         t = -prevT
#         # thruster fire time bounds
#         t[t > 1] = 1
#         t[t < 0.1] = 0
#         dt = np.amax(t)
#     else:
#         print("SOMETHING WRONG IN PID FUNCTION!")

#     return t, dt                                      # function returns the PID output

def pid(nav, ref):   
    global intError, lastError, prevT

    # pose error
    error = nav - ref 
                         
    # store control inputs
    u = np.zeros(3)

    # distance vector
    dis = np.array([error[0], error[1]])

    if np.linalg.norm(dis) > 0.03 or np.abs(error[2]*pi/180) > 5*pi/180:
        # # pid error terms
        # intError += error*dt                  # compute integral
        # rateError = (error - lastError)/dt    # compute derivative

        # pid output
        # u = kp*error #+ ki*intError + kd*rateError 
        u[0] = kpp*error[0]
        u[1] = kpp*error[1]
        u[2] = kpy*error[2]
                 
        # # store previous error
        # lastError = error
        
        # location of each matric on floatbot
        # from serc gnc paper
        thruster_matrix = np.array([[-1, 1, 0, 0, 1, -1, 0, 0], 
                                    [0, 0, 1, -1, 0, 0, -1, 1], 
                                    [-1, 1, -1, 1, -1, 1, -1, 1]])
        
        # psudoinverse of thruster matrix
        mInv = np.linalg.pinv(thruster_matrix)
        
        # time for each thruster
        t = 2*np.matmul(mInv, u)#.round(decimals=2) # dont need to round since bounds takes care of it below

        # thruster fire time bounds
        t[t > 1] = 1
        t[t < 0.1] = 0
    else:
        print("SOMETHING WRONG IN PID FUNCTION!")

    return t 
""" PID CONTROLLER """


""" THRUST CONTROL """
# thruster gpio pins
thrusters = [17, 27, 22, 10, 9, 11, 0, 5]

# setup thruster gpio pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(thrusters, GPIO.OUT, initial=GPIO.HIGH)  # relays OFF - reverse logic for some reason

def fire_thruster(pin, name, fireTime):
    # dt = fireTime if fireTime >= 0.1 else 0 - handled in pid thread already
    
    GPIO.output(pin, GPIO.LOW)
    # print(name, 'ON')
    time.sleep(fireTime)
    GPIO.output(pin, GPIO.HIGH)
    # print(name, 'OFF')

def thruster_control(fireTimes):

    # create a list of threads
    thruster_threads_list = []

    # initialize and start threads - takes 3-4ms to do
    for i in range(8):
        t = threading.Thread(target=fire_thruster, 
                             name="thruster{}".format(i+1), 
                             args=(thrusters[i],"thruster{}".format(i+1),fireTimes[i]))
        thruster_threads_list.append(t)     # append threads into list
        t.start()   # start thread
        # print(t.name, "has started")

    # continue control loop after every thruster fired
    for t in thruster_threads_list:
        t.join()
""" THRUST CONTROL """

# NEED TO WRITE SOME USER INTERFACE CODE HERE
# reference point
ref = [0.92, 0.63, 90]     # [meter, meter, deg]

# control loop parameters
# h = 0.2   # 5Hz control loop 


print("STARTING CONTROL LOOP")
try:
    while True:
        # testing naviation thread - WORKS BUT USB PORT OF THE MOBILE BEACON CHANGES
        # print("inside control loop")
        print("x: ", floatbot_pose[0], "y: ", floatbot_pose[1], "yaw: ", floatbot_pose[2])

        # compute firing times
        # t, dt = pid(floatbot_pose, ref)
        t = pid(floatbot_pose, ref)

        # thruster control
        thruster_control(t)

        # when at goal fire all thrusters then break out of control loop
        # if np.linalg.norm(dis) < 0.03 or np.abs(error[2]*pi/180) < 5*pi/180:
        #     t =  np.array([[0.1], [0.1], [0.1], [0.1], [0.1], [0.1], [0.1], [0.1]])     # pose from gps and imu

        # time.sleep(dt+1)
        time.sleep(3)
        # time.sleep(1)

except KeyboardInterrupt:
    GPIO.cleanup()
    print("CLEANED UP GPIO")
    time.sleep(1)
    print('TERMINATED CONTROL LOOP')