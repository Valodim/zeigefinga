import numpy
import sys

import bufs

class StreamInput(object):
    def __init__(self, f, numbuf=False):
        self.shape = None
        self.buf = None
        self.numbuf = numbuf
        self.f = f

    def prototype(self, num=3, sep=' ', **kwargs):
        if self.shape is not None:
            return False

        # prototype matches
        s = None
        # number of successful matches
        j = 1
        for i in range(num*10):
            # read the first line to determine number of input fields
            inp = self.f.readline()
            if not inp:
                return False
            inp = numpy.fromstring(inp.strip(), sep=sep, **kwargs)

            # if it matches the old shape, count one up
            if s == inp.shape:
                j += 1
            # otherwise - reset to new prototype format
            else:
                j = 1
                s = inp.shape

            # if we have enough, return true
            if j >= num:
                # we got a shape prototype!
                self.shape = inp.shape
                if self.numbuf:
                    self.buf = bufs.arrbuf(length=self.numbuf, shape=self.shape)
                print >> sys.stderr, "using prototype format: ", inp
                return True

        # no prototype shape found!
        return False

    def __iter__(self):

        while True:

            inp = self.f.readline()
            # break on eol
            if not inp:
                break

            # skip markers
            try:
                inp = numpy.fromstring(inp, sep=' ')
                if inp.shape != self.shape:
                    raise ValueError()

                # return or buffer
                if not self.buf:
                    yield inp
                else:
                    # put in buffer, and yield mean value if return value is true
                    if self.buf.put(inp):
                        yield self.buf.get()

            except ValueError:
                print >> sys.stderr, "bad input line:", inp

