from mpu6050 import mpu6050
from time import sleep, time

mpu = mpu6050(0x68)
mpu.set_gyro_range(0x00)
range = mpu.read_gyro_range(raw = False)
print(range)


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
g5 = 0
g4 = 0
g3 = 0
g2 = 0
g1 = 0
gyro_data = 0.0         # set initial gyro rate
gyroUpdateRate = 0.005   # 50ms == 20Hz update rate
prev_gryo_data = 0.0
h = 0.003

start = time()
try:
    while True:
        getTime = time()
        # compute angle at current timestep

        # compute angle at current timestep
        g5 = g4
        g4 = g3
        g3 = g2
        g2 = g1
        g1 = prev_gryo_data
        g = (g1+g2+g3+g4+g5)/5      # avg last 5 gyro readings
        yawAng += g*h*2

        print(yawAng)

        # store prev gyro rate
        gyro_data = mpu.get_gyro_data()
        prev_gryo_data = gyro_data['z']-gyroZBias
        endTime = time()

        h = endTime - getTime
        sleep(h)

except KeyboardInterrupt:
    print('exit gyro process')
""" pull gyro readings """
