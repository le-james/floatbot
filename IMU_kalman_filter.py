import numpy as np
from mpu6050 import mpu6050
import time



""" gyro model state space """

deltaT = 0.01  # integration timestep [sec]

A = np.array([[1.0, -deltaT], [0.0, 1.0]])
B = np.array([[deltaT], [0.0]])
""" gyro model state space """

Vd = 0.001*np.eye(2)  # disturbance covariance - A model uncertainty
Vn = 0.03             # noise covariance - y sensor uncertainty

C_sens = np.array([[1, 0]])   #1x2

def dlqr_calculate(G, H, Q, R, returnPE=False):
    '''
    Discrete-time Linear Quadratic Regulator calculation.
    State-feedback control  u[k] = -K*x[k]

    How to apply the function:    
        K = dlqr_calculate(G,H,Q,R)
        K, P, E = dlqr_calculate(G,H,Q,R, return_solution_eigs=True)

    Inputs:
        G, H, Q, R  -> all numpy arrays  (simple float number not allowed)
        returnPE: define as True to return Ricatti solution and final eigenvalues

    Returns:
        K: state feedback gain
        P: Ricatti equation solution
        E: eigenvalues of (G-HK)  (closed loop z-domain poles)
    '''
    from scipy.linalg import solve_discrete_are, inv, eig
    P = solve_discrete_are(G, H, Q, R)  #Ricatti solution
    K = inv(H.T@P@H + R)@H.T@P@G        #K = (B^T P B + R)^-1 B^T P A 

    if returnPE == False:   return K    #one line if statement - exits if false

    from numpy.linalg import eigvals
    eigs = np.array([eigvals(G-H@K)]).T
    return K, P, eigs

# discrete ricacati solution for kalman filter gains
dL, dPL, deigsL = dlqr_calculate(np.transpose(A), np.transpose(C_sens), Vd, Vn, True)   # set to true to return K, P and eigs





""" start reading gyro yaw """
mpu = mpu6050(0x68)

# set initial yaw angle
yawAng = 0.0
prevBias = 0.0

# xhat = np.array([[yawAng], [0.2]])
pred = np.array([[yawAng], [prevBias]])
xhat = np.array([[yawAng], [prevBias]])
prevXHat = np.array([[yawAng], [prevBias]])

Pnn = np.zeros((2,2))
W = np.array([[0.01, 0], [0.0, 0.2]])
V = Vn


# the scalar kalman filter parameters
# angle = 0
# bias = -0.2
# prevRate = 0

""" pull gyro readings """
start = time.time()
try:
    while True:
        t_since_epoch = round(time.time() - start)
        # print(t_since_epoch)

        # 100hz pull rate
        getTime = time.time()
        gyro_data = mpu.get_gyro_data()
        time.sleep(0.007)       # time = 1/100 - 0.003
        endTime = time.time()

        dt = endTime-getTime    # 3ms to get gyro data <- 1/3ms = 333Hz



        yawAng = yawAng + (gyro_data['z']+0.21)*dt

        print(yawAng)


        """ another way to write the kalman filter """
        # # predict
        # print("delta t: ", dt)
        # print("next loop prevrate: ",prevRate)
        # rate = prevRate - bias
        # print("predict rate: ", rate)
        # angle += dt * rate
        # print("predict angle: ", angle)

        # # measure
        # err = (gyro_data['z']-bias)*dt - angle  # very small value
        # print("measure err: ", err)
        
        # # update
        # angle += dL[0][0]*err     # very small value
        # print("update angle: ", angle)
        # bias += dL[0][1]*err      # around 0.21
        # print("update bias: ", bias)

        # # previous rate
        # prevRate = gyro_data['z']
        # print("store prev rate: ",prevRate)
        # print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        """ another way to write the kalman filter - doesn't work """



        """ doesn't work either """
        # B = np.array([[dt], [0.0]])

        # # prediction
        # pred = np.matmul(A, prevXHat) + (B*(gyro_data['z'] + xhat[1][0]))
        # # predict covariance
        # Pn = np.linalg.multi_dot([A, Pnn, np.transpose(A)]) + W

        # # kalman gain
        # L = np.matmul(Pn, np.transpose(C_sens)) * np.linalg.inv(np.linalg.multi_dot([C_sens, Pn, np.transpose(C_sens)]) + V)

        # time.sleep(deltaT)

        # getTime2 = time.time()
        # gyro_data2 = mpu.get_gyro_data()
        # endTime2 = time.time()

        # dt2 = endTime2-getTime2    # 3ms to get gyro data <- 1/3ms = 333Hz


        # # measure
        # y = (gyro_data2['z'] + xhat[1][0])*dt2

        # # update
        # xhat = pred + np.matmul(L, (y - np.matmul(C_sens, pred))) 
        # # xhat = (pred[0][0] + dL[0][0]*(((gyro_data['z']+0.2)*dt) - pred[0][0]))  
        # # prevXHat = np.array([[xhat], [gyro_data['z']]])     # store estimated state for next prediction in next loop
        # prevXHat = xhat
        # # update covariance
        # Pnn = np.matmul(np.eye(2) - np.matmul(L, C_sens), Pn)

        # print(xhat)
        """ doesn't work either """


except KeyboardInterrupt:
    print('interrupted!')
""" pull gyro readings """

""" start reading gyro yaw """