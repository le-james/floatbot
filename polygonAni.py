import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
import matplotlib.patches as patches
import matplotlib as mpl

# fig = plt.figure()
# ax = fig.add_subplot(111)
# ax.set_xlim(-30,30)
# ax.set_ylim(-30,30)

fig = plt.figure()
fig.set_dpi(100)
fig.set_size_inches(5, 4.5)
ax = plt.axes(xlim=(-30, 30), ylim=(-30, 30))

v = np.array([[-5, -10], [-10, -5], [-10, 5], [-5, 10], [5, 10], [10, 5], [10, -5], [5, -10]])

patch = patches.Polygon(v, closed=True, fill=False, ec="green")

ax.add_patch(patch)

def init():
    return patch,

def animate(i):
    v[:,0]+=i
    v[:,1]+=i
    patch.set_xy(v)
    print(i)
    return patch,

ani = animation.FuncAnimation(fig, animate, frames=4, init_func=init, interval=2000, repeat=False)
plt.grid(True)
plt.show()










# import matplotlib.pyplot as plt
# import numpy as np
# import matplotlib.animation as animation
# import matplotlib.patches as patches

# fig = plt.figure()
# ax = fig.add_subplot(111)
# # ax.set_xlim(-10,10)
# # ax.set_ylim(-10,10)
# ax.set_xlim(-1,5)
# ax.set_ylim(-1,5)

# P1x=[0.0,0.5,1.0,1.5,2.0,2.5,3.0]
# P1y=[0.0,0.0,0.0,0.0,0.0,0.0,0.0]
# P2x=[1.0,1.5,2.0,2.5,3.0,3.5,4.0]
# P2y=[0.0,0.0,0.0,0.0,0.0,0.0,0.0]
# P3x=[1.0,1.5,2.0,2.5,3.0,3.5,4.0]
# P3y=[1.0,1.0,1.0,1.0,1.0,1.0,1.0]
# P4x=[0.0,0.5,1.0,1.5,2.0,2.5,3.0]
# P4y=[1.0,1.0,1.0,1.0,1.0,1.0,1.0]

# P = np.concatenate((np.array([P1x, P2x, P3x, P4x]).reshape(4,1,len(P1x)),
#                     np.array([P1y, P2y, P3y, P4y]).reshape(4,1,len(P1x))), axis=1)

# patch = patches.Polygon(P[:,:,0],closed=True, fc='r', ec='r')
# ax.add_patch(patch)

# print(P[0,0,3])

# def init():
#     return patch,

# def animate(i):
#     patch.set_xy(P[:,:,i])
#     return patch,

# # ani = animation.FuncAnimation(fig, animate, np.arange(P.shape[2]), init_func=init,
# #                               interval=1000, blit=True, repeat=False)
# plt.show()