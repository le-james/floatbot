from mpu6050 import mpu6050
import time
import numpy as np
import csv

mpu = mpu6050(0x68)

# number of seconds to pull gyro data for
t = 120

# gyro values
gyro_vals = np.zeros((3, t+1))
# gyro_mean_offset = np.zeros(3)


""" pull gyro readings """
print("PULLING GYRO READING AND STORING THEM FOR:", t, "seconds...")
start = time.time()
while True:
    t_since_epoch = round(time.time() - start)
    print(t_since_epoch)

    gyro_data = mpu.get_gyro_data()
    # print(gyro_data['z'])

    gyro_vals[0, t_since_epoch] = gyro_data['x']
    gyro_vals[1, t_since_epoch] = gyro_data['y']
    gyro_vals[2, t_since_epoch] = gyro_data['z']

    time.sleep(1)
    if t_since_epoch == t:
        break
""" pull gyro readings """



# print(gyro_vals)



""" averaging elements of an array """
print("COMPUTING MEAN OFFSET OF ALL GYRO AXIS...")
meanX_offset = sum(gyro_vals[0, :]) / len(gyro_vals[0, :]) 
meanY_offset = sum(gyro_vals[1, :]) / len(gyro_vals[1, :]) 
meanZ_offset = sum(gyro_vals[2, :]) / len(gyro_vals[2, :]) 
gyro_mean_offset = [meanX_offset, meanY_offset, meanZ_offset]
""" averaging elements of an array """



""" create and write gyro vals and mean offset to cvs file """
print("WRITING GYRO READINGS AND OFFSET TO CVS FILE...")
with open ("gyro_bias_6.csv", "w", newline="") as csvfile:

    # column names
    fieldnames = ["time", "x gyro vals", "x gyro mean offset", 
                          "y gyro vals", "y gyro mean offset", 
                          "z gyro vals", "z gyro mean offset"]

    # write column names in cvs file
    thewriter = csv.DictWriter(csvfile, fieldnames=fieldnames)
    thewriter.writeheader()

    # fill in columns
    for i in range(t_since_epoch):
        thewriter.writerow({"time": i, "x gyro vals": gyro_vals[0, i], "x gyro mean offset": gyro_mean_offset[0],
                                       "y gyro vals": gyro_vals[1, i], "y gyro mean offset": gyro_mean_offset[1],
                                       "z gyro vals": gyro_vals[2, i], "z gyro mean offset": gyro_mean_offset[2]})
""" create and write gyro vals and mean offset to cvs file """
