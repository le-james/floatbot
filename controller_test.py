from cmath import pi
import numpy as np

""" SIMPLE TRANSLATIONAL CONTROLLER """
def xy_controller(nav, ref):   

    # pose error
    error = nav - ref                      

    # 1-norm (error)
    dis_error = np.array([error[0], error[1]])

    # psudo control action
    u = dis_error

    # xy thruster matrix reaction force
    thruster_matrix = np.array([[-1, 1, 0, 0, 1, -1, 0, 0], 
                                [0, 0, 1, -1, 0, 0, -1, 1]])
    
    # psudoinverse of thruster matrix
    mInv = np.linalg.pinv(thruster_matrix)
    
    # compute the inverse of u=mt, where m is the thruster matrix
    t = np.matmul(mInv, u)  # dont need to multiply by 2

    # thrusters to fire
    t[t<0] = 0
    t[t>0] = 0.1

    return t    # function returns the PID output
""" SIMPLE TRANSLATIONAL CONTROLLER """

""" SIMPLE YAW CONTROLLER """
def yaw_controller(nav, ref):   

    # pose error
    error = nav - ref                      

    # distance vector
    yaw_error = error[2]

    # psudo control action
    u = yaw_error

    # yaw thruster matrix reaction torque
    thruster_matrix = np.array([[-1, 1, -1, 1, -1, 1, -1, 1]])
    
    # psudoinverse of thruster matrix
    mInv = np.linalg.pinv(thruster_matrix)
    
    # compute the inverse of u=mt, where m is the thruster matrix
    t = np.matmul(mInv, u)  # dont need to multiply by 2

    # thrusters to fire
    t[t<0] = 0
    t[t>0] = 0.05

    return t    # function returns the PID output
""" SIMPLE YAW CONTROLLER """

# test output of controller
nav = np.array([[1], [1], [45]])    # pose from gps and imu
ref = np.array([[0], [0], [0]])     # where we want to go

t = xy_controller(nav, ref)
print(t)
t = yaw_controller(nav, ref)
print(t)