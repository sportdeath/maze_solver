import numpy as np


class CircularArray(object):
    """ A simple circular array implementation 

        You can append any number of elements, but only the last N will be kept
        where N is the integer passed to the constructor. Useful for smoothing values.
    """
    def __init__(self, size):
        self.arr = np.zeros(size)
        self.ind = 0
        self.num_els = 0

    def append(self, value):
        if self.num_els < self.arr.shape[0]:
            self.num_els += 1
        self.arr[self.ind] = value
        self.ind = (self.ind + 1) % self.arr.shape[0]

    # returns the mean of maintained elements
    def mean(self):
        return np.mean(self.arr[:self.num_els])

    # returns the median of maintained elements
    def median(self):
        return np.median(self.arr[:self.num_els])
