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
fields = [ int(x)-1 for x in sys.argv[1:] ] if len(sys.argv) > 1 else None 
# get a stream input object from sys.stdin
stream = StreamInput(sys.stdin, fields=fields)
# prototype from three consecutive lines of equal shape
if not stream.prototype(1):
    sys.exit(1)
gen = iter(stream)

device = uinput.Device([
    uinput.REL_X,
    uinput.REL_Y,
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
    if len(inp) <= 2 or inp[2] == 1:
        device.emit(uinput.REL_X, int(inp[0]), syn=False)
        device.emit(uinput.REL_Y, -int(0.8*inp[1]))

    return True

gobject.io_add_watch(sys.stdin, gobject.IO_IN, update_line)
gobject.MainLoop().run()

