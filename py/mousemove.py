import matplotlib
matplotlib.use('GTKAgg')

import sys
import gobject
import numpy
import time
import uinput

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

device = uinput.Device([
    uinput.REL_X,
    uinput.REL_Y,
    uinput.ABS_X + (0, 255, 0, 0),
    uinput.ABS_Y + (0, 255, 0, 0),
    uinput.BTN_LEFT,
    uinput.BTN_RIGHT
    ])
time.sleep(0.3)

def update_line(*args):
    global gen

    inp = gen.next()
    if inp is False:
        print >> sys.stderr, "reached EOF or error in input"
        sys.exit(0)

    print int(inp[1]), int(inp[0])
    if False: # or int(cbuf.get()[-1,6]) is 1:
        print "!"
        device.emit(uinput.ABS_X, 127)
        device.emit(uinput.ABS_Y, 127)
    else:
        device.emit(uinput.REL_X, int(inp[0]), syn=False)
        device.emit(uinput.REL_Y, -int(0.8*inp[1]))

    return True

gobject.io_add_watch(sys.stdin, gobject.IO_IN, update_line)
gobject.MainLoop().run()

