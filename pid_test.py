from cmath import pi
import numpy as np

""" PID CONTROLLER """
# pid parameters
kp = 0.75
ki = 1
kd = 1
intError = 0
lastError = 0
prevT = np.zeros(8)

def pid(nav, ref):   
    global intError, lastError, prevT

    # pose error
    error = nav - ref                      
    error[2] = error[2]*pi/180

    # distance vector
    dis = np.array([error[0], error[1]])

    if np.linalg.norm(dis) > 0.05 or np.abs(error[2]) > 5*pi/180:
        # # pid error terms
        # intError += error*dt                  # compute integral
        # rateError = (error - lastError)/dt    # compute derivative

        # pid output
        u = kp*error #+ ki*intError + kd*rateError          

        # # store previous error
        # lastError = error
        
        # location of each matric on floatbot
        thruster_matrix = np.array([[-1, 1, 0, 0, 1, -1, 0, 0], 
                                    [0, 0, 1, -1, 0, 0, -1, 1], 
                                    [-1, 1, -1, 1, -1, 1, -1, 1]])
        
        # psudoinverse of thruster matrix
        mInv = np.linalg.pinv(thruster_matrix)
        
        # time for each thruster
        t = 2*np.matmul(mInv, u)#.round(decimals=2)

        # thruster fire time bounds
        t[t > 1] = 1
        t[t < 0.1] = 0

        # store prev t
        prevT = t

        # longest thruster time
        dt = np.amax(t)
        # flag = 1
    elif np.linalg.norm(dis) < 0.05 and np.abs(error[2]) < 5*pi/180:
        t = -prevT
        # thruster fire time bounds
        t[t > 1] = 1
        t[t < 0.1] = 0
        dt = np.amax(t)
    else:
        print("SOMETHING WRONG IN PID FUNCTION!")

    return t, dt                                      # function returns the PID output
""" PID CONTROLLER """


nav = np.array([[1], [0], [0]])     # pose from gps and imu
ref = np.array([[0], [0], [0]])     # where we want to go

t, dt = pid(nav, ref)

print(t)
# print(dt)


# kp = 1.5

# u = kp*np.array([[0], [0], [0.79]])   # kp*err

# thruster_matrix = np.array([[-1, 1, 0, 0, 1, -1, 0, 0], 
#                             [0, 0, 1, -1, 0, 0, -1, 1], 
#                             [-1, 1, -1, 1, -1, 1, -1, 1]])

# # psudoinverse of thruster matrix
# mInv = np.linalg.pinv(thruster_matrix)

# times = 2*np.matmul(mInv, u).round(decimals=2)
# print(times)