import rospy
#from yaml import load

import time


class Timer:
	def __init__(self, smoothing):
		self.arr = CircularArray(smoothing)
		self.last_time = time.time()

	def tick(self):
		t = time.time()
		self.arr.append(1.0 / (t - self.last_time))
		self.last_time = t

	def fps(self):
		return self.arr.mean()