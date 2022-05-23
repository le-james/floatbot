import time
# Import matplotlib, numpy and math
import matplotlib.pyplot as plt
import numpy as np
  


x = 1
y = 5
x = 10

u[0] = x





# # thruster gpio pins
# thrusters = [17, 27, 22, 10, 9, 11, 5, 6]

# for i in thrusters:
#     print(i)


# fireTimes = np.array([[1], [0], [0], [0], [0], [0], [0], [0]])
# print(fireTimes[1][0])

# flag = 1

# while flag:
#     print("hello")
#     flag = 0


# x = np.linspace(-10, 10, 100)
# z = 1/(1 + np.exp(-x))
# # c1 = 0.5
# # c2 = 0.3
# # z = 1/(1 + np.exp(-c1*(x-c2)))
  
# plt.plot(x, z)
# plt.xlabel("x")
# plt.ylabel("Sigmoid(X)")
  
# plt.show()



# a = [1, 2 ,3]

# print(a)



# u = 10
# a = u if u > 0.01 else 0
# print(a)

# floatbot_pose = zeros(3)
# floatbot_pose[0] = 2
# floatbot_pose[1] = 5
# print(floatbot_pose)

# leds = [5, 6, 12, 13, 16, 26]

# for led in leds:
#     print(led)



# arr = np.array([11, 12, 13, 14, 15, 16, 17, 15, 11, 12, 14, 15, 16, -17, 20])

# print(arr-5)

# A = np.amax(arr)
# print(A)
# B = np.amax(A)
# print(B)

# gyro_vals = np.zeros((1, 420))

# print(gyro_vals[0, 10])



# xhat = np.array([[5], [9]])

# print(xhat[0][0])



# print(np.zeros((2,2)))
# print(np.eye(2))



# print(np.array([[0.001, 0], [0.0, 0.003]]))

# a = np.array([[1, 2], [3, 4]])
# b = np.array([[1, 2], [3, 4]])
# c = np.array([[1, 2], [3, 4]])

# d = a@b@c
# print(d)
# e = a.dot(b).dot(c)
# print(d)
# f = np.linalg.multi_dot([a, np.transpose(b), c])
# print(f)



# a = np.array([[5], [9]])
# print(np.shape(np.array([[1, 0]])))
# r = 6
# D = np.array([[1], [3]])

# print(np.matmul(D, a))
# print(D*a)
# print(a*2)
# print(np.shape(a[1]))

# b = [a[1]+a[1]]
# p = np.array([b, [r]])

# print(b[0][0])
# print(np.array([9]))



# start = time.time()
# print("Give the IMU some time")
# time.sleep(0.001)
# end = time.time()
# print("Starting gyro calibration process... wait time:",end - start, "sec")




# start = time.time()
# while True:
#     t_since_epoch = round(time.time() - start)
#     print(t_since_epoch)
#     time.sleep(1)

#     if t_since_epoch == 5:
#         break

# print(t_since_epoch)




# a = np.zeros((3, 6))
# a[0, 0] = 5
# a[1, 1] = 5
# a[2, 2] = 5

# b = a[0, :]
# c = [70, 9]
# print(c)


# averaging elements of an array
# def Average(arr): 
#     avg = sum(a[0, :]) / len(a[0, :]) 
#     return avg
# avg = sum(a[0, :]) / len(a[0, :])
# print(avg)



# getting time step
# t0 = time.time()
# i = 0
# while True:
#     # tk = time.time()
#     # time.sleep(0.01)
#     # tkn = time.time()
#     # dt = tkn - tk
#     # print(dt)

#     time.sleep(0.5)

#     i += i + 1
#     print(i)


#     if time.time()-t0 > 5:
#         break

# print(i)











# # import keyboard
# import math
# import auto_diff
# import numpy as np

# import hellowrld

# hellowrld.helloworld



# r = np.array([0, 0, 45])
# v = np.array([0, 0, 0])
# x = np.append(r, v)
# q = np.zeros(3)

# r = np.array([[1], [1], [0]])
# v = np.array([[0], [0], [0]])
# v = np.zeros((3, 1))
# x = np.concatenate((r, v), axis=0)

# q = np.zeros((3, 1))
# print(q)





# # x = np.array([2, 3])
# x = np.array([[2], [3]])
# u = np.array([[5], [5]])

# # print(x)
# # print(y)
# # r = x*x + np.array([x[1], x[1]])
# # print(r)

# def testFunc(x, u):
#     z = x*x + np.array([x[1], x[1]])*np.array([x[1], x[1]]) + u*u
#     return z


# print(testFunc(x))

# def linearize_Sys(x):
#     with auto_diff.AutoDiff(x) as x:
#         f_eval = testFunc(x, u)
#         z, Jf = auto_diff.get_value_and_jacobian(f_eval)
#     return z, Jf

# with auto_diff.AutoDiff(x, u) as (x, u):
#     f_eval = testFunc(x, u)
#     z, (A, B) = auto_diff.get_value_and_jacobians(f_eval)

# z, Jf = linearize_Sys(x)


# print(f_eval)
# print(z)
# print('A matrix: ')
# print(A)
# print('B matrix: ')
# print(B)





# c = np.cos(np.deg2rad(77))
# print(c)
# print(int(1.9)) #rounds down



# a = np.array([[1, 2, 3], [1, 2, 3]])

# print(a[1][2])




# print("hello")

# while True:
#     if keyboard.read_key() == "p":
#         print("You pressed p")
#         break