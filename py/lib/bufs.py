import numpy

class arrbuf():
    """ 
    
    Simple fixed size ndarray buffer for mean value calculation. Takes elements
    until full,.provides mean of accumulated values.
    
    Example usage:

        buf = arrbuf(length=100, shape=[2])

        while True:
            # get data. must be shape=[2]
            data = getData()

            # if this returns true, buffer is full!
            if buf.put(data):

                # print mean of collected data. this clears the buffer as well
                print buf.get()

    """

    def __init__(self, length=100, shape=[4]):
        shape = list(shape)
        shape.insert(0, length)
        print shape
        self.buf = numpy.ndarray(shape=shape)
        self.i = 0
        self.length = length

    def put(self, line):
        """ appends an element, returns True is full, False otherwise """
        assert self.i <= self.length
        if self.i == self.length:
            raise Exception

        self.buf[self.i] = line
        self.i += 1

        return self.i == self.length

    def get(self):
        if not self.i == self.length:
            return False

        self.i = 0

        return self.buf.mean(0)

class circbuf():
    
    """
    Simple fixed size circular ndarray buffer. Takes up to n elements, after
    which the least recently added element is replaced.
    
    Example usage:

        buf = circbuf(length=100, shape=[2])

        while True:
            # get data. must be shape=[2].
            data = getData()

            # just put the data
            buf.put(data):

            if random(chance=0.01):
                # returns the most recently added 100 results
                print buf.get()

    """

    def __init__(self, length=100, shape=[4]):
        shape = list(shape)
        shape.insert(0, length*2)
        self.buf = numpy.empty(shape=shape)
        self.i = 0
        self.length = length

    def put(self, line):
        self.buf[self.i] = line
        self.buf[self.i+self.length] = line

        self.i = (self.i+1)%self.length

    def get(self):
        return self.buf[self.i:self.i+self.length]


