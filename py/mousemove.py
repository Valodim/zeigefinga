import matplotlib
matplotlib.use('GTKAgg')

import sys
import gobject
import numpy
import time
import uinput

import lib.bufs as bufs


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
    global buf, cbuf

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

    print int(cbuf.get()[-1,4]), int(cbuf.get()[-1,5]), int(cbuf.get()[-1,6])
    if int(cbuf.get()[-1,6]) is 1:
        print "!"
        device.emit(uinput.ABS_X, 127)
        device.emit(uinput.ABS_Y, 127)
    else:
        device.emit(uinput.REL_X, -int(cbuf.get()[-1,5]), syn=False)
        device.emit(uinput.REL_Y, -int(0.8*cbuf.get()[-1,4]))

    update_line.cnt += 1
    return True

update_line.cnt = 0

gobject.io_add_watch(sys.stdin, gobject.IO_IN, update_line)
gobject.MainLoop().run()

