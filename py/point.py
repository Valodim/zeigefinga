import matplotlib
matplotlib.use('GTKAgg')

import sys
import gobject
import pylab as p
import numpy
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
gen = iter(stream)

ax = p.subplot(111)
canvas = ax.figure.canvas
ax.set_ylim(-1000, 1000)
ax.set_xlim(-1000, 1000)

circ = p.Circle((50,50), 20, color='r', animated=True)
ax.add_patch(circ)

background = None

def update_line(*args):
    global gen, background, circ

    inp = gen.next()
    if inp is False:
        print >> sys.stderr, "reached EOF or error in input"
        sys.exit(0)

    if background is None:
        background = canvas.copy_from_bbox(ax.bbox)

    # restore the clean slate background
    canvas.restore_region(background)

    # reset into the middle
    # if inp[-1] == 1:
        # circ.center = (0,0)
    # elif abs(inp[5]) > 2 and abs(inp[4]) > 2:
    # else:
    circ.center = (circ.center[0] -inp[1], circ.center[1] + inp[0])
    ax.draw_artist(circ)

    # just redraw the axes rectangle
    canvas.blit(ax.bbox)

    return True

gobject.io_add_watch(sys.stdin, gobject.IO_IN, update_line)
p.show()
