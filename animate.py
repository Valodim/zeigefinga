import matplotlib
matplotlib.use('GTKAgg')

import sys
import gobject
import pylab as p
import numpy
import time

import lib.bufs as bufs

ax = p.subplot(111)
canvas = ax.figure.canvas

# for profiling
tstart = time.time()

# read the first line to determine number of input fields
inp = sys.stdin.readline()
if not inp:
    sys.exit(0)
print >> sys.stderr, "using prototype format: ", inp
inp = numpy.fromstring(inp.strip(), sep=' ')

buf = bufs.arrbuf(length=2, shape=inp.shape)
cbuf = bufs.circbuf(shape=inp.shape)
shortbuf = bufs.circbuf(length=15, shape=inp.shape)

# set y axis limits
ax.set_ylim(-1100, 1100)

# create the initial line
x = p.arange(0, 100)
plots = { }
if len(sys.argv) > 1:
    for i in sys.argv[1:]:
        print i
        plots[int(i)], = p.plot(x, p.empty(100), animated=True)
else:
    for i in range(0, inp.shape[0]):
        plots[i], = p.plot(x, p.empty(100), animated=True)

# save the clean slate background -- everything but the animated line
# is drawn and saved in the pixel buffer background
background = None

minimum = 0

def update_line(*args):
    global buf, cbuf, background, plots
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

    newmean = buf.get()
    cbuf.put(newmean)
    shortbuf.put(newmean)

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

    update_line.cnt += 1
    return True

update_line.cnt = 0

def trololo():
    gobject.io_add_watch(sys.stdin, gobject.IO_IN, update_line)
    return False

gobject.idle_add(trololo)

# gobject.idle_add(update_line)
p.show()
