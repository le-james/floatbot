from mpu6050 import mpu6050
from time import sleep, time

mpu = mpu6050(0x68)



""" GYRO Z-AXIS CALIBRATION CODE """
print("Calibrating Gyro Yaw (z-axis)... ")
count = 0
gyroReadings = 1000     # number of gyro data to pull
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
        sleep(1)
        print("Gyro yaw bias (z-axis) is: ", gyroZBias)
        break
""" GYRO Z-AXIS CALIBRATION CODE """

# break to read calibration details
print("1 seconds before starting gyro integration")
sleep(1)

""" pull gyro readings """
# gyro inital conditions
yawAng = 0.0            # set initial yaw angle
gyro_data = 0.0         # set initial gyro rate
gyroUpdateRate = 0.05   # 50ms == 20Hz update rate
prev_gryo_data = 0.0

start = time()
try:
    while True:
        getTime = time()
        # compute angle at current timestep
        yawAng += (prev_gryo_data-gyroZBias)*gyroUpdateRate
        print(yawAng)

        # get gyro rate
        gyro_data = mpu.get_gyro_data()
        prev_gryo_data = gyro_data['z']
        endTime = time()
        dt = endTime-getTime        # approx 3ms to get gyro data <- 1/3ms = 333Hz
        h = gyroUpdateRate - dt     # time taken out of timestep
        sleep(h)               # pause loop until next timestep

except KeyboardInterrupt:
    print('exit gyro process')
""" pull gyro readings """
