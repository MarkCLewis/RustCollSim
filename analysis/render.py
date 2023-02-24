import numpy as np
import more_itertools
import matplotlib.pyplot as plt

with open('demo_big_sim_hills_sliding_brick.csv') as f:
    data = [[float(d) for d in e.strip().split() if d]
            for e in f.readlines()]

    # now this is a list of list of particles (particle is a list of 4 floats)
    data = [list(more_itertools.chunked(row, 4)) for row in data]


sxy = 1e-5


plt.figure(figsize=[5, 5])
ax = plt.axes([0.1, 0.1, 0.8, 0.8], xlim=(-sxy/2, sxy/2), ylim=(-sxy/2, sxy/2))


points_whole_ax = 5 * 0.8 * 72    # 1 point = dpi / 72 pixels
def points_radius(radius): return (2 * radius / sxy * points_whole_ax) ** 2


for d in data:

    x = [p[0] for p in d]
    y = [p[1] for p in d]
    radii = [p[3] for p in d]
    ax.scatter(x, y, s=[points_radius(r) for r in radii], color='r')

plt.grid()

# https://stackoverflow.com/questions/14896580/matplotlib-hooking-in-to-home-back-forward-button-events

plt.show()
