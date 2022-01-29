# import keyboard
import math
import auto_diff
import numpy as np

import hellowrld

hellowrld.helloworld



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

def testFunc(x, u):
    z = x*x + np.array([x[1], x[1]])*np.array([x[1], x[1]]) + u*u
    return z


# print(testFunc(x))

# def linearize_Sys(x):
#     with auto_diff.AutoDiff(x) as x:
#         f_eval = testFunc(x, u)
#         z, Jf = auto_diff.get_value_and_jacobian(f_eval)
#     return z, Jf

with auto_diff.AutoDiff(x, u) as (x, u):
    f_eval = testFunc(x, u)
    z, (A, B) = auto_diff.get_value_and_jacobians(f_eval)

# z, Jf = linearize_Sys(x)


print(f_eval)
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


