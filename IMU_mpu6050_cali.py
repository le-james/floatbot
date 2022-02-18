from mpu6050 import mpu6050

mpu = mpu6050(0x68)



""" GYRO Z-AXIS CALIBRATION CODE """

""" pull gyro readings """
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
        print("Gyro yaw bias (z-axis) is: ", gyroZBias)
        break
""" pull gyro readings """

""" GYRO Z-AXIS CALIBRATION CODE """