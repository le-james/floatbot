from html import entities
from time import sleep, time
from mpu6050 import mpu6050

mpu = mpu6050(0x68)

""" GYRO Z-AXIS CALIBRATION CODE """
""" pull gyro readings """
print("Calibrating Gyro Yaw (z-axis)... ")
count = 0
gyroReadings = 420     # number of gyro data to pull
sumZGyroReadings = 0
gyroZBias = 0
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
        sleep(0.5)    
        print("Gyro yaw bias (z-axis) is: ", gyroZBias)
        break
""" pull gyro readings """
""" GYRO Z-AXIS CALIBRATION CODE """

# break to read calibration details
print("5 seconds before starting gyro integration")
sleep(5)

""" pull gyro readings """
# set initial yaw angle
yawAng = 0
gyroUpdateRate = 0.05   # 50ms
dt = gyroUpdateRate
prevYawRate = 0
print("Starting gyro... ")
try:
    while True:
        startTime = time()                     # start loop time
        gyro_data = mpu.get_gyro_data()        # get yaw rate
        yawAng += (gyro_data['z']+gyroZBias)*dt   # yaw angle integration
        # prevYawRate = gyro_data['z']           # store yaw rate for next time step  
        print(yawAng)
        endTime = time()                      # time after getting rate

        dtGyro = endTime - startTime          # gyro process time
        # print(dtGyro)

        sleep(gyroUpdateRate - dtGyro)          # wait until next time step

except KeyboardInterrupt:
    print('Exited angle readings')
""" pull gyro readings """