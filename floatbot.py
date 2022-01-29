import auto_diff
import numpy as np
# import autograd.numpy as np  # Thinly-wrapped numpy
# from autograd import grad    # The only autograd function you may ever need

# sim parameters
h = 0.05            # time step
t = 10              # sim time
t_steps = int(t/h)  # number of time steps

"""     Dynamics of the FloatBot:

        x is the states of the system
        u is the control inputs to the system (fx, fy, torque(tau))

            x_Dot = vx*cos(theta) - vy*sin(theta) =>  velocity in x direction
            y_Dot = vx*sin(theta) + vy*cos(theta) =>  velocity in y direction
        theta_Dot = omega                         =>  angular velocity about z axis
           vx_Dot = fx/m - omega*vx               =>  acceleration in x direction
           vy_Dot = fy/m - omega*vy               =>  acceleration in y direction
        omega_Dot = tau/Iz                        =>  angular acceleration about z axis
 """


# floatbot parameters
m = 1       # mass
Izz = 10    # moment of inertia about the z direction 

# x = np.array([1,2,3,4,5,6])
# u = np.array([10,10,1])

def floatbot_Dynamics(x, u):
    
    # states
    theta = x[2]    # angular position - degrees
    vx = x[3]       # velocity in x direction
    vy = x[4]       # velocity in y direction
    omega = x[5]    # angular velocity

    # controls
    fx = u[0]       # applied force in the x axis
    fy = u[1]       # applied force in the y axis
    tau = u[2]      # applied torque about z axis

    # kinematics
    x_Dot = vx*np.cos(np.deg2rad(theta)) - vy*np.sin(np.deg2rad(theta))
    y_Dot = vx*np.sin(np.deg2rad(theta)) + vy*np.cos(np.deg2rad(theta))
    theta_Dot = omega

    # dynamics
    vx_Dot = fx/m - omega*vx
    vy_Dot = fy/m - omega*vy
    omega_Dot = tau/Izz

    # next states of the floatbot given x and u
    xn = np.array([x_Dot, y_Dot, theta_Dot, vx_Dot, vy_Dot, omega_Dot])

    return xn

# # test dynamics outputs - outputs correct values, compared with my MATLAB code
# st = floatbot_Dynamics(x, u)
# # np will output float numbers instead of scientific notation
np.set_printoptions(formatter={'float_kind':'{:f}'.format})  
# print(st)

# RK4 integration with zero-order hold on u
def floatbot_Dynamics_rk4(x,u):
    f1 = floatbot_Dynamics(x, u)
    f2 = floatbot_Dynamics(x + 0.5*h*f1, u)
    f3 = floatbot_Dynamics(x + 0.5*h*f2, u)
    f4 = floatbot_Dynamics(x + h*f3, u)
    xn = x + (h/6.0)*(f1 + 2*f2 + 2*f3 + f4)
    return xn

# def linearize_Sys(x, u):
#     x = np.reshape(x, (6, 1))
#     u = np.reshape(u, (3, 1))

#     with auto_diff.AutoDiff(x) as x:
#         f_eval = floatbot_Dynamics_rk4(x, u)
#         y, A = auto_diff.get_value_and_jacobian(f_eval)
#     return y, A


# reference and A linearization point
r = np.array([0, 0, 45])    # pose
v = np.array([0, 0, 0])     # velocity
x = np.reshape(np.append(r, v), (6, 1))

# control and B linearization point
u = np.reshape(np.zeros(3), (3, 1))

# doesn't work - look at floatbot-R1 for working auto diff
with auto_diff.AutoDiff(x) as x:
    f_eval = floatbot_Dynamics_rk4(x, u)
    y, A = auto_diff.get_value_and_jacobian(f_eval)



# y, A = linearize_Sys(x, u)
# print(y)
print('A matrix: ')
print(A)
# print('B matrix: ')
# print(B)

# print(x)
# print(u)
# print(np.cos(np.deg2rad([45])) - np.cos(np.deg2rad([45])))










# #truster matrix - look at diagram in SERC GNC paper
# m = np.array([[-1, 1, 0, 0, 1, -1, 0, 0],
#               [ 0, 0, 1, -1, 0, 0,-1, 1],
#               [-1, 1,-1, 1, -1, 1, -1, 1]])

# print(m)



#pseudoinverse
# a= np.array([[1,2,3,4], [4,5,6,7]])
# p = np.linalg.pinv(a)
# print(p)





#sim

# #initial conditions
# x0 = np.zeros(6)
# u0 = np.zeros(3)

# xhist = np.zeros((np.size(x0), t_steps))
# uhist = np.zeros((np.size(u0), t_steps))

# print(np.size(uhist[1]))





