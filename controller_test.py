from cmath import pi
import numpy as np

""" SIMPLE CONTROLLER """
def controller(nav, ref):   

    # pose error
    error = nav - ref                      
    error[2] = error[2]*pi/180

    # distance vector
    dis = np.array([error[0], error[1]])

    # psudo control action
    u = dis

    # location of each matric on floatbot
    thruster_matrix = np.array([[-1, 1, 0, 0, 1, -1, 0, 0], 
                                [0, 0, 1, -1, 0, 0, -1, 1]])
    
    # psudoinverse of thruster matrix
    mInv = np.linalg.pinv(thruster_matrix)
    
    # time for each thruster
    t = 2*np.matmul(mInv, u)#.round(decimals=2)

    # thrusters to fire
    t[0 if condition else b] = 1

    return t    # function returns the PID output
""" SIMPLE CONTROLLER """

nav = np.array([[1], [1], [0]])     # pose from gps and imu
ref = np.array([[0], [0], [0]])     # where we want to go

t = controller(nav, ref)

print(t)