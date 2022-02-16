from mpu6050 import mpu6050
import time
import numpy as np


mpu = mpu6050(0x68)

# set initial yaw angle
yawAng = 0
prevBias = 0

""" pull gyro readings """
start = time.time()
try:
    while True:
        t_since_epoch = round(time.time() - start)
        # print(t_since_epoch)

        getTime = time.time()

        gyro_data = mpu.get_gyro_data()

        endTime = time.time()

        # calibrated gyro readings
        # print(gyro_data['x']-6.389)   
        # print(gyro_data['y']-2.095)   
        # print(gyro_data['z']+0.319)   

        dt = endTime-getTime    # 3ms to get gyro data <- 1/3ms = 333Hz

        yawAng += (gyro_data['z']+0.2)*dt
        # yawAng += (gyro_data['z']+0.23)*dt - prevBias*dt
        # prevBias = gyro_data['z']

        print(yawAng)
        # print(dt)


except KeyboardInterrupt:
    print('interrupted!')
""" pull gyro readings """
