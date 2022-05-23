from marvelmind import MarvelmindHedge
from mpu6050 import mpu6050
import numpy as np
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



""" FLOATBOT NAVIGATION TEST """
# gyro inital conditions
yawAng = 0.0            # set initial yaw angle
gyro_data = 0.0         # set initial gyro rate
gyroUpdateRate = 0.01   # 50ms == 20Hz update rate
prev_gryo_data = 0.0

# position and angle of floatbot
floatbot_pose = np.zeros(3)

#Define GPS Hedgehog object, need to check the port
hedge = MarvelmindHedge(tty = "/dev/ttyACM1", adr = None, debug = False)
print("Starting GPS...")
hedge.start()

print("STARTING NAV")
time.sleep(1)

try:
    while True:
        startTime = time.time()

        # get GPS position and parse
        pos = hedge.position()
        x = pos[1]
        y = pos[2]

        # compute angle at current timestep
        yawAng += prev_gryo_data*gyroUpdateRate

        # store prev gyro rate
        gyro_data = mpu.get_gyro_data()
        prev_gryo_data = gyro_data['z']-gyroZBias

        # store pose in array
        floatbot_pose[0] = x
        floatbot_pose[1] = y
        floatbot_pose[2] = yawAng
        print("x: ", floatbot_pose[0], "y: ", floatbot_pose[1], "yaw: ", floatbot_pose[2])
        
        endTime = time.time()

        time.sleep(endTime - startTime + 1)    # pause loop until next timestep

except KeyboardInterrupt:
    print("exiting nav loop")

""" FLOATBOT NAVIGATION TEST """