import matplotlib
matplotlib.use('GTKAgg')

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

import sys
import gobject
import pylab as p
import numpy
import time

import lib.bufs as bufs

ax = p.subplot(111, projection='3d')
canvas = ax.figure.canvas

# for profiling
tstart = time.time()

# read the first line to determine number of input fields
inp = sys.stdin.readline()
if not inp:
    sys.exit(0)
print >> sys.stderr, "using prototype format: ", inp
inp = numpy.fromstring(inp.strip(), sep=' ')

buf = bufs.arrbuf(length=15, shape=inp.shape)

ax.set_xlim(1000, -1000)
ax.set_ylim(-1000, 1000)
ax.set_zlim(-1000, 1000)

# create the initial line
plots = { }
plots[0], = ax.plot([0, inp[0]], [0, inp[1]], zs=[0, inp[2]])
# plots[1], = ax.plot_surface([0, inp[0]], [0, inp[1]], zs=[0, inp[2]])

# save the clean slate background -- everything but the animated line
# is drawn and saved in the pixel buffer background
background = None

minimum = 0

def update_line(*args):
    global buf
    global minimum

    inp = sys.stdin.readline()
    if not inp:
        sys.exit(0)
    # do nothing, yet
    if inp == "--MARKER--\n":
        print >> sys.stderr, "found a marker, ignoring."
        return True

    inp = numpy.fromstring(inp, sep=' ')
    if not buf.put(inp):
        return True

    inp = buf.get()

    # update the data
    # print inp
    plots[0].set_xdata([0,inp[0]])
    plots[0].set_ydata([0,inp[1]])
    plots[0].set_3d_properties(zs=[0,inp[2]])

    ax.draw_artist(plots[0])
    # just draw the animated artist
    # ax.set_ylim(ydata.max())

    # just redraw the axes rectangle
    # canvas.blit(ax.bbox)
    plt.draw()

    update_line.cnt += 1

    return True

update_line.cnt = 0

def trololo():
    gobject.io_add_watch(sys.stdin, gobject.IO_IN, update_line)
    return False

gobject.idle_add(trololo)
# gobject.idle_add(update_line)

p.show()
