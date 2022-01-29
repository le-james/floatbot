import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
import matplotlib.patches as patches
import matplotlib as mpl

fig = plt.figure()
fig.set_dpi(100)
fig.set_size_inches(5, 4.5)
ax = plt.axes(xlim=(-10, 10), ylim=(-10, 10))

rad = np.deg2rad(35)

patch = patches.RegularPolygon((0, 0), 5, radius=5, orientation=0)


def init():
    ax.add_patch(patch)
    return patch,

def animate(i):
    x, y = patch.center
    x = 5 + 3 * np.sin(np.radians(i))
    y = 5 + 3 * np.cos(np.radians(i))
    patch.center = (x, y)
    return patch,

ani = animation.FuncAnimation(fig, animate, frames=4, init_func=init, interval=2000, repeat=False)

plt.grid(True)
plt.show()
