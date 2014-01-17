import matplotlib
matplotlib.use('GTKAgg')

import sys
import gobject
import pylab as p
import time

import lib.bufs as bufs
from lib.streaminput import StreamInput

ax = p.subplot(111)
canvas = ax.figure.canvas

# get a stream input object from sys.stdin
stream = StreamInput(sys.stdin)
# prototype from three consecutive lines of equal shape
if not stream.prototype(1):
    sys.exit(1)

cbuf = bufs.circbuf(shape=stream.shape)

# set y axis limits
ax.set_ylim(-100, 100)

# create the initial line
x = p.arange(0, 100)
plots = { }
if len(sys.argv) > 1:
    for i in sys.argv[1:]:
        print i
        plots[int(i)], = p.plot(x, p.empty(100), animated=True)
else:
    for i in range(0, stream.shape[0]):
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
