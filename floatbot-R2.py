from cmath import pi
import auto_diff
# import control
from control.matlab import *    # allows us to just call lqr()
import numpy as np


np.set_printoptions(formatter={'float_kind':'{:f}'.format}) 
use_numpy_matrix(flag=False, warn=True) 



""" sim andfloatbot parameters """

h = 0.01
t = 50
t_steps = int(t/h)

m = 1       # mass [kg]
Izz = 10    # moment of inertia about the z direction [kgm2]

""" sim andfloatbot parameters """



""" floatbot dynamics (ODEs) """

def floatbot_Dynamics(x, u):
    # np.deg2rad() gives error if i put it in here for some reason

    # states
    theta = x[2]*pi/180     # angular position - degrees
    vx = x[3]               # velocity in x direction
    vy = x[4]               # velocity in y direction
    omega = x[5]            # angular velocity

    # controls
    fx = u[0]       # applied force in the x axis
    fy = u[1]       # applied force in the y axis
    tau = u[2]      # applied torque about z axis

    # kinematics
    x_Dot = vx*np.cos(theta) - vy*np.sin(theta)     
    y_Dot = vx*np.sin(theta) + vy*np.cos(theta)
    theta_Dot = omega

    # dynamics
    vx_Dot = fx/m - omega*vx
    vy_Dot = fy/m - omega*vy
    omega_Dot = tau/Izz

    # next continuous states of the floatbot given x and u
    xn = np.array([x_Dot, y_Dot, theta_Dot, vx_Dot, vy_Dot, omega_Dot])

    return xn

""" floatbot dynamics (ODEs) """



""" RK4 integrator, linearization and riccati """

#RK4 integration with zero-order hold on u
def floatbot_Dynamics_rk4(x,u):
    f1 = floatbot_Dynamics(x, u)
    f2 = floatbot_Dynamics(x + 0.5*h*f1, u)
    f3 = floatbot_Dynamics(x + 0.5*h*f2, u)
    f4 = floatbot_Dynamics(x + h*f3, u)
    xn = x + (h/6.0)*(f1 + 2*f2 + 2*f3 + f4)
    return xn

# reference and A linearization point
r = np.array([100, 100, 40])             # pose
v = np.array([0, 0, 0])                 # velocity
x = np.reshape(np.append(r, v), (6, 1))

# control and B linearization point
u = np.reshape(np.zeros(3), (3, 1))

def continousSys(x, u):
    with auto_diff.AutoDiff(x, u) as (x, u):
        f_eval = floatbot_Dynamics(x, u)
        cz, (cA, cB) = auto_diff.get_value_and_jacobians(f_eval)
    return cz, cA, cB
cz, cA, cB = continousSys(x, u)

def discreteSys(x, u):
    with auto_diff.AutoDiff(x, u) as (x, u):
        f_eval = floatbot_Dynamics_rk4(x, u)
        dz, (dA, dB) = auto_diff.get_value_and_jacobians(f_eval)
    return dz, dA, dB
dz, dA, dB = discreteSys(x, u)

Q = np.eye(6)   # state weights
R = np.eye(3)   # control weights

#continuous ricacati solution
cK, cS, cE = lqr(cA, cB, Q, R)    # A and B doesn't take into account the zero order hold from integration

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

# discrete ricacati solution
dK, dP, deigs = dlqr_calculate(dA, dB, Q, R, True)   # set to true to return K, P and eigs 

""" RK4 integrator, linearization and riccati """



""" LQR controller """

# controllability matrix and rank
# P = ctrb(dA, dB)
# print("Controllability matrix rank: ", np.linalg.matrix_rank(P), "==", np.size(x), ", ctrb if equal")

def controller(x):
    rNext = x[0:3]
    vNext = x[3:7]

    deltaX = np.reshape(np.append(rNext - r, vNext- v), (6, 1)) # shape (6,1)

    u = np.reshape(-np.matmul(dK, deltaX), 3)  # numpy matrix multiplication - shape (3,)

    return u

""" LQR controller """



""" Kalman filter """

Vd = 0.1*np.eye(6)  # disturbance covariance - model uncertainty
Vn = np.eye(3)      # noise covariance - sensor uncertainty

C_sens = [[1, 0, 0, 0, 0, 0],
          [0, 1, 0, 0, 0, 0],
          [0, 0, 1, 0, 0, 0]]

C = np.eye(6)

C_pose = [[1, 0, 0, 0, 0, 0],
          [0, 1, 0, 0, 0, 0],
          [0, 0, 1, 0, 0, 0],
          [0, 0, 0, 0, 0, 0],
          [0, 0, 0, 0, 0, 0],
          [0, 0, 0, 0, 0, 0]]

# observability matrix and rank
# Q = obsv(dA, C)
# print("Observability matrix rank: ", np.linalg.matrix_rank(Q), "==", np.size(x), ", obsv if equal")

# discrete ricacati solution for kalman filter gains
dL, dPL, deigsL = dlqr_calculate(np.transpose(dA), np.transpose(C_sens), Vd, Vn, True)   # set to true to return K, P and eigs 

# ddL, ddP, ddE = lqe(dA, Vd, C_sens, Vd, Vn)

""" Kalman filter """



""" floatbot sim """

# initial conditions
x0 = np.zeros(6)
u0 = np.zeros(3)

# storing sim state and control inputs
uhist = np.zeros((np.size(u0), t_steps))    # control history
plant = np.zeros((np.size(x0), t_steps+1))  # acting as the actual system=
sens = np.zeros((np.size(x0), t_steps+1))   # acting as the sensor measurement - initial condition is same as plant
xhat = np.zeros((np.size(x0), t_steps+1))   # estimated states vx,vy,omega

# xhist[:, 0] = [0, 0, 0, 0, 0, 0]  # change initial condition 

for x in range(t_steps):
    uhist[:,x] = controller(plant[:,x]) #+ np.random.randn(6)*0.75)                                 
    # pretend this is the actual plant with external disturbances/model uncertainty/process noise etc.
    plant[:,x+1] = floatbot_Dynamics_rk4(plant[:,x], uhist[:,x]) + np.random.randn(6)*0.75
    sens[:,x+1] = np.matmul(C_pose, plant[:,x+1] + np.random.randn(6)*0.75)                     # sensor noise
    xhat[:,x+1] = plant[:,x+1] + np.matmul(np.transpose(dL), (sens[0:3,x+1] - np.matmul(C_sens, plant[:,x+1])))  # kalman filter

print(plant[:,60])
print()
print(sens[:,60])
print()
print(xhat[:,60])

# print(dL)
# print()
# print(ddL)
# print()
# print(np.transpose(dL))




# for i in xhist:
#     for j in i:
#         print(j, end=" ")
#     print()

""" floatbot sim """



""" floatbot state plots """

# import plotly.graph_objects as go

# t = np.linspace(0, t_steps, num=t_steps, dtype=int)
# fig = go.Figure()
# fig.add_trace(go.Scatter(x=t, y=xhist[0,t], mode='lines', name='x position'))
# fig.add_trace(go.Scatter(x=t, y=xhist[1,t], mode='lines', name='y position'))
# fig.add_trace(go.Scatter(x=t, y=xhist[2,t], mode='lines', name='theta position'))

# fig.update_layout(title='Floatbot States vs Time',
#                    xaxis_title='Time',
#                    yaxis_title='States')

# fig.show()

""" floatbot state plots """



""" floatbot animation """

# from turtle import *
# import turtle

# screen = Screen()
# screen.setup(500, 500)

# # turtle object
# floatbot = turtle.Turtle()

# # turtle paramenters
# floatbot.shapesize(2, 2, 2)
# floatbot.pensize(2)

# # 1-10 speed range
# floatbot.speed(0)

# for i in range(t_steps):
#     floatbot.setheading(xhist[2, i])
#     floatbot.goto(xhist[0, i], xhist[1, i])

# turtle.done()

""" floatbot animation """