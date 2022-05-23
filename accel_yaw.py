from cmath import asin
from mpu6050 import mpu6050
import time 
import numpy as np

mpu = mpu6050(0x68)
mpu.set_accel_range(0x00)
range = mpu.read_accel_range(raw = False)
print("Accel Set to Range: ", range)

r = 0.22    # [m]
accelUpdateRate = 0.05   # 10ms == 100Hz update rate

# def yawAngleCalc(x):
#     rate = 2*asin(x/2*r)
#     return rate

# def rk4(xk, h):
#     f1 = yawAngleCalc(xk)
#     f2 = yawAngleCalc(xk + 0.5*h*f1)
#     f3 = yawAngleCalc(xk + 0.5*h*f2)
#     f4 = yawAngleCalc(xk + h*f3)
#     return (h/6.0)*(f1 + 2*f2 + 2*f3 + f4)

try:
    while True:
        getTime = time.time()

        # integrate
 
        # get accel data
        accelerometer_data = mpu.get_accel_data()
        accelerometer_data['y']

        





        endTime = time.time()



        dt = endTime-getTime        # approx 3ms to get gyro data <- 1/3ms = 333Hz
        h = accelUpdateRate - dt     # time taken out of timestep
        time.sleep(h)                    # pause loop until next timestep

except KeyboardInterrupt:
    print('exit accel process')
""" pull accel readings """




