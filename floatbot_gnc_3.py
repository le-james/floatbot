from marvelmind import MarvelmindHedge
from mpu6050 import mpu6050
import RPi.GPIO as GPIO
from cmath import pi
import numpy as np
import threading
import time
import keyboard


""" set up imu gyro """
mpu = mpu6050(0x68)
mpu.set_gyro_range(0x00)
""" set up imu gyro """



""" GYRO Z-AXIS CALIBRATION CODE """
print("Calibrating Gyro Yaw (z-axis)... ")
count = 0
gyroReadings = 420     # number of gyro data to pull
sumZGyroReadings = 0
while True:     # calibrate gyro
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
        floatbot_pose[0] = x*100    # convert to cm
        floatbot_pose[1] = y*100    # convert to cm
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



""" SIMPLE TRANSLATIONAL CONTROLLER """
def xy_controller(nav, ref):   

    # pose error
    error = nav - ref                      

    # 1-norm (error)
    dis_error = np.array([error[0], error[1]])

    # psudo control action
    u = dis_error

    # xy thruster matrix reaction force
    thruster_matrix = np.array([[-1, 1, 0, 0, 1, -1, 0, 0], 
                                [0, 0, 1, -1, 0, 0, -1, 1]])
    
    # psudoinverse of thruster matrix
    mInv = np.linalg.pinv(thruster_matrix)
    
    # compute the inverse of u=mt, where m is the thruster matrix
    t = np.matmul(mInv, u)  # dont need to multiply by 2

    # thrusters to fire
    t[t<0] = 0
    t[t>0] = 0.1

    return t    # function returns the PID output
""" SIMPLE TRANSLATIONAL CONTROLLER """

""" SIMPLE YAW CONTROLLER """
def yaw_controller(nav, ref):   

    # pose error
    error = nav - ref                      

    # distance vector
    yaw_error = error[2]

    # psudo control action
    u = yaw_error

    # yaw thruster matrix reaction torque
    thruster_matrix = np.array([[-1, 1, -1, 1, -1, 1, -1, 1]])
    
    # psudoinverse of thruster matrix
    mInv = np.linalg.pinv(thruster_matrix)
    
    # compute the inverse of u=mt, where m is the thruster matrix
    t = np.matmul(mInv, u)  # dont need to multiply by 2

    # thrusters to fire
    t[t<0] = 0
    t[t>0] = 0.05

    return t    # function returns the PID output
""" SIMPLE YAW CONTROLLER """



""" THRUST CONTROL """
# thruster gpio pins
thrusters = [17, 27, 22, 10, 9, 11, 0, 5]

# setup thruster gpio pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(thrusters, GPIO.OUT, initial=GPIO.HIGH)  # relays OFF - reverse logic for some reason

def fire_thruster(pin, name, fireTime):
    
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



""" USER SET POINT """
ref = [0, 0, 0]

def isfloat(num):
    try:
        float(num)
        return True
    except ValueError:
        return False

print("Enter where the Floatbot should go and press enter...")
ref[0] = input("x position [cm]: ")
ref[1] = input("y position [cm]: ")
ref[2] = input("yaw angle [deg]: ")

check = ref     # copy shape of ref

for idx, val in enumerate(ref):
    check[idx] = isfloat(val)   
# print("check ", check)
# print(all(check))

# checks if user input is valid
# userInCheck = True   # in user input mode
def setpoint():    # user set the goal pose of the floatbot
    while True:
        for idx, val in enumerate(check):
            if val == False:
                if idx == 0:
                    print("Re-enter x postion again...")
                    ref[idx] = input("x position [cm]: ")
                    check[idx] = isfloat(ref[idx])   
                elif idx == 1:
                    print("Re-enter y postion again...")
                    ref[idx] = input("y position [cm]: ")
                    check[idx] = isfloat(ref[idx])   
                elif idx == 2:
                    print("Re-enter yaw angle again...")
                    ref[idx] = input("yaw angle [deg]: ")
                    check[idx] = isfloat(ref[idx])   

        if all(check) == True:
            # convert into float and meter values
            ref[0] = float(ref[0])/100
            ref[1] = float(ref[1])/100
            ref[2] = float(ref[2])
            # print out where the floatbot will move to
            print("The Floatbot will move to: " + "(" , ref[0], "," , ref[1], ")" 
                    + " with angle: ", ref[2], " [deg]")
            # userInCheck = False
            break
setpoint()
""" USER SET POINT """



""" CONTROL LOOP """
# control loop parameters
h = 0.2   # 5Hz control loop 

print("Press the 'tab' key to start control loop")
keyboard.wait('tab')
print("STARTING CONTROL LOOP...")

try:
    while True:
        # testing naviation thread - WORKS BUT USB PORT OF THE MOBILE BEACON CHANGES
        # print("inside control loop")
        print("x: ", floatbot_pose[0], "y: ", floatbot_pose[1], "yaw: ", floatbot_pose[2])

        # translational motion
        t = xy_controller(floatbot_pose, ref)
        thruster_control(t)

        # yaw motion
        t = yaw_controller(floatbot_pose, ref)
        thruster_control(t)




        time.sleep(3)
except KeyboardInterrupt:
    GPIO.cleanup()
    print("CLEANED UP GPIO")
    time.sleep(1)
    print('TERMINATED CONTROL LOOP')
""" CONTROL LOOP """