from cmath import pi
from turtle import shape
import auto_diff
# import control
from control.matlab import *    # allows us to just call lqr()
import numpy as np

np.set_printoptions(formatter={'float_kind':'{:f}'.format})  

# sim parameters
h = 0.05
t = 15
t_steps = int(t/h)

#floatbot parameters
m = 1       # mass
Izz = 10    # moment of inertia about the z direction 

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

#RK4 integration with zero-order hold on u
def floatbot_Dynamics_rk4(x,u):
    f1 = floatbot_Dynamics(x, u)
    f2 = floatbot_Dynamics(x + 0.5*h*f1, u)
    f3 = floatbot_Dynamics(x + 0.5*h*f2, u)
    f4 = floatbot_Dynamics(x + h*f3, u)
    xn = x + (h/6.0)*(f1 + 2*f2 + 2*f3 + f4)
    return xn

# reference and A linearization point
r = np.array([100, -100, 0])           # pose
v = np.array([0, 0, 0])                 # velocity
x = np.reshape(np.append(r, v), (6, 1))
# x = np.array([[0], [0], [np.pi], [0], [0], [0]])

# control and B linearization point
u = np.reshape(np.zeros(3), (3, 1))
# u = np.array([[0], [0], [0]])

def continousSys(x, u):
    with auto_diff.AutoDiff(x, u) as (x, u):
        f_eval = floatbot_Dynamics(x, u)
        cz, (cA, cB) = auto_diff.get_value_and_jacobians(f_eval)
    return cz, cA, cB
cz, cA, cB = continousSys(x, u)
# print(cA)

def discreteSys(x, u):
    with auto_diff.AutoDiff(x, u) as (x, u):
        f_eval = floatbot_Dynamics_rk4(x, u)
        dz, (dA, dB) = auto_diff.get_value_and_jacobians(f_eval)
    return dz, dA, dB
dz, dA, dB = discreteSys(x, u)
# print(dA)

# print(f_eval)
# # print(z)
# print('A matrix: ')
# print(A)
# print('B matrix: ')
# print(B)

Q = np.eye(6)   # state weights
R = np.eye(3)   # control weights

#continuous ricacati solution
cK, cS, cE = lqr(cA, cB, Q, R)    # A and B doesn't take into account the zero order hold from integration
# print(cK)

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

#discrete ricacati solution
dK, dP, deigs = dlqr_calculate(dA, dB, Q, R, True)   # set to true to return K, P and eigs 
# print(dK)

# print(np.matmul(-dK, [0, 0, 5, 0, 0, 0]))
# print(np.shape(np.reshape(np.matmul(-dK, [0, 0, 5, 0, 0, 0]), (3, 1))))
# conint = np.matmul(-dK, [0, 0, 5, 0, 0, 0])

def controller(x):
    rNext = x[0:3]
    vNext = x[3:7]

    deltaX = np.reshape(np.append(rNext - r, vNext- v), (6, 1)) # shape (6,1)

    u = np.reshape(-np.matmul(dK, deltaX), 3)  # numpy matrix multiplication - shape (3,)

    return u

# rNext = x[0:3]
# vNext = x[3:7]
# deltaX = np.array([rNext - r, vNext - v])
# print(np.shape(deltaX))
# print(np.shape(dK))
# testest = np.reshape(np.append(rNext, vNext), (6, 1))
# print(np.shape(testest))

# testx = x[:,0][0:3]
# print(x[:,0][3:7])
# print(np.array([testx - r, [1, 1, 1]]))
# print(r)


# Kalman filter

C = [1, 1, 1, 0, 0, 1]

xhat = dA*xhat_k + dB*u_k + L*(y - C*xhat_k)










# floatbot sim

# initial conditions
x0 = np.zeros(6)
u0 = np.zeros(3)

# storing sim state and control inputs
xhist = np.zeros((np.size(x0), t_steps+1))
uhist = np.zeros((np.size(u0), t_steps))

# xhist[:, 0] = [0, 0, 0, 10, 10, 0]

# print(np.shape(uhist[:,0]))
# print(np.shape(conint))
# uhist[:,0] = conint
# print(uhist[:,0:3])

# xhist[:,0] = x0
# print(xhist)
# print(xhist.shape)
# print(uhist.shape)
# print(uhist[:,0].shape)

for x in range(t_steps):
    uhist[:,x] = controller(xhist[:,x] + np.random.rand(6))   # added state purterbations for fun
    xhist[:,x+1] = floatbot_Dynamics_rk4(xhist[:,x], uhist[:,x])

# print(np.shape(xhist))
# print(xhist)
# print(uhist)
# uu = controller(xhist[:,0])
# print(uu.shape)
# print(np.reshape(uu, 3).shape)
# print(np.shape(controller(xhist[:,0])))

# for i in xhist:
#     for j in i:
#         print(j, end=" ")
#     print()
























# floatbot animation

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
# floatbot.speed(3)

# for i in range(t_steps):
#     floatbot.goto(xhist[0, i], xhist[1, i])
#     floatbot.setheading(xhist[2, i])

# turtle.done()