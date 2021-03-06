from mpu6050 import mpu6050
from time import sleep, time
import numpy as np

mpu = mpu6050(0x68)
mpu.set_gyro_range(0x00)
range = mpu.read_gyro_range(raw = False)

print(range)

""" GYRO Z-AXIS CALIBRATION CODE """
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
        gyroZBias = np.sum(gyro_z_vals) / np.size(gyro_z_vals) 
        zth = np.amax(gyro_z_vals)    # max bias in array
        print("Read", np.size(gyro_z_vals), "gyro readings")
        sleep(1)
        print("Avg gyro yaw bias (z-axis) is: ", gyroZBias)
        sleep(1)
        print("Largest absolute gyro rate val: ", zth)
        break
""" GYRO Z-AXIS CALIBRATION CODE """


sleep(1)
# break to read calibration details
print("3 seconds before starting gyro integration")
sleep(3)


""" pull gyro readings """
# gyro inital conditions
yawAng = 0.0            # set initial yaw angle
prevYawAng = 0.0        
gyro_data = 0.0         # set initial gyro rate
gyroUpdateRate = 0.05   # 10ms == 100Hz update rate
prev_gryo_data = 0.0

start = time()
try:
    while True:
        startTime = time()

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
        endTime = time()

        # h = gyroUpdateRate - (endTime - startTime)     # time taken out of timestep

        h = endTime - startTime

        sleep(h)    # pause loop until next timestep

except KeyboardInterrupt:
    print('exit gyro process')
""" pull gyro readings """
