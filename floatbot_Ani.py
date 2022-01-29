import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation

#plot setup
fig = plt.figure()
fig.set_dpi(100)
fig.set_size_inches(5, 4.5)
ax = plt.axes(xlim=(0, 100), ylim=(0, 100))

#drawing
enemy = plt.Circle((10, 10), 0.75, fc='r')
agent = plt.Circle((15, 10), 0.75, fc='b')

# pts = np.array([[20,20], [40,20], [50, 5]])
# pts = np.array([[-5, -10], [-10, -5], [-10, 5], [-5, 10], [5, 10], [10, 5], [10, -5], [5, -10]])

# p = plt.Polygon(pts, closed=True, fill=False, hatch="|", color="blue")

# print(p.get_xy())

def init():
    #alt method to initialize position of enemy and agent
    # enemy.center = (5, 5)
    # agent.center = (5, 10)
    ax.add_patch(agent)
    ax.add_patch(enemy)
    # ax.add_patch(p)
    return []

init()  #puts the dots on the plot - use to see one frame for testing

#animation 1
def animateLine(i, patch):
    x, y = patch.center
    x += 0.25
    y += 0.25
    patch.center = (x, y)
    return patch

#animation 2
def animateCos(i, patch):
    x, y = patch.center
    x += 0.2
    y = 50 + 30 * np.cos(np.radians(i))
    patch.center = (x, y)
    return patch

#package the animations into one function
def animationManage(i,agent,enemy):
    animateCos(i,enemy)
    animateLine(i,agent)
    return []

anim = animation.FuncAnimation(fig, animationManage,
                               init_func=init,
                               frames=275,
                               fargs=(agent,enemy),
                               interval=20,
                               blit=True,
                               repeat=False)


plt.show()



























# fig = plt.figure()
# fig.set_dpi(100)
# fig.set_size_inches(5, 4.5)

# ax = plt.axes(xlim=(0, 100), ylim=(0, 100))
# enemy = plt.Circle((10, -10), 0.75, fc='r')
# agent = plt.Circle((10, -10), 0.75, fc='b')


# def init():
#     enemy.center = (5, 5)
#     agent.center = (5, 10)
#     ax.add_patch(agent)
#     ax.add_patch(enemy)

#     return []

# init()  #puts the dots on the plot

# def animationManage(i,agent,enemy):
#     animateCos(i,enemy)
#     animateLine(i,agent)
#     return []


# def animateLine(i, patch):
#     x, y = patch.center
#     x += 0.25
#     y += 0.25
#     patch.center = (x, y)
#     return patch,


# def animateCos(i, patch):
#     x, y = patch.center
#     x += 0.2
#     y = 50 + 30 * np.cos(np.radians(i))
#     patch.center = (x, y)
#     return patch,

# anim = animation.FuncAnimation(fig, animationManage,
#                                init_func=init,
#                                frames=360,
#                                fargs=(agent,enemy,),
#                                interval=20,
#                                blit=True,
#                                repeat=True)


# plt.show()