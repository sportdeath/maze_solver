from circular_array import CircularArray
import time

class Timer:
    ''' A simple timer class to track iterations per second

        Uses a CircularArray to smooth FPS values.
        Pass an integer to indicate how many values to maintain.
    '''
    def __init__(self, smoothing):
        self.arr = CircularArray(smoothing)
        self.last_time = time.time()

    ''' Call this on every iteration
    '''
    def tick(self):
        t = time.time()
        self.arr.append(1.0 / (t - self.last_time))
        self.last_time = t

    ''' Call this to check recent average calls per second 
    '''
    def fps(self):
        return self.arr.mean()
