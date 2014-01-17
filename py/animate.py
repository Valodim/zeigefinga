import matplotlib
matplotlib.use('GTKAgg')

import sys
import gobject
import pylab as p
import time

import lib.bufs as bufs
from lib.streaminput import StreamInput

# remap input fields?
fields = map(lambda x: int(x)-1, sys.argv[1:]) if len(sys.argv) > 1 else None
# get a stream input object from sys.stdin
stream = StreamInput(sys.stdin, fields=fields)
# prototype from three consecutive lines of equal shape
if not stream.prototype(1):
    sys.exit(1)

# circular buffer holds 100 elements for the y-axis
cbuf = bufs.circbuf(shape=stream.shape)

# set up plotting stuff
ax = p.subplot(111)
canvas = ax.figure.canvas
ax.set_ylim(-100, 100)

# create the initial line
x = p.arange(0, 100)
plots = { }
for i in range(stream.shape[0]):
    plots[i], = p.plot(x, p.empty(100), animated=True)

# save the clean slate background -- everything but the animated line
# is drawn and saved in the pixel buffer background
background = None

gen = iter(stream)

def update_line(*args):
    global gen, cbuf, background, plots

    inp = gen.next()
    if inp is False:
        print >> sys.stderr, "reached EOF or error in input"
        sys.exit(0)

    cbuf.put(inp)

    if background is None:
        background = canvas.copy_from_bbox(ax.bbox)

    # restore the clean slate background
    canvas.restore_region(background)

    # update the data
    for i in plots:
        ydata = cbuf.get()[:,i]
        plots[i].set_ydata(ydata)
        ax.draw_artist(plots[i])
    # just draw the animated artist
    # ax.set_ylim(ydata.max())
    # just redraw the axes rectangle
    canvas.blit(ax.bbox)

    return True

def trololo():
    gobject.io_add_watch(sys.stdin, gobject.IO_IN, update_line)
    return False

gobject.idle_add(trololo)

# gobject.idle_add(update_line)
p.show()
