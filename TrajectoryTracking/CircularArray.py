import rospy
import numpy as np
from yaml import load
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Vector3, Point, Pose, PoseStamped, PoseArray, Quaternion, PolygonStamped, Polygon, Point32, PoseWithCovarianceStamped, PointStamped
from std_msgs.msg import Header, ColorRGBA
from nav_msgs.msg import OccupancyGrid

import json, time, collections, recordclass

import tf.transformations
import tf

import skimage.morphology
from scipy import ndimage


EPSILON = 0.00000000001

''' These data structures are used in the search function
'''

Circle = collections.namedtuple("Circle", ["radius", "center", "angle", "deflection"])
Path  = collections.namedtuple("Path", ["states"])
# wraps a state with additional search information
SearchNode = collections.namedtuple("SearchNode", ["state", "cost", "heuristic", "parent"])
SearchNodeTree = collections.namedtuple("SearchNodeTree", ["state", "cost", "heuristic", "parent", "tree_node"])
# used for recreating the search tree
TreeNode = recordclass.recordclass("TreeNode", ["state", "children"])

class CircularArray(object):
	"""docstring for CircularArray"""
	def __init__(self, size):
		self.arr = np.zeros(size)
		self.ind = 0
		self.num_els = 0

	def append(self, value):
		if self.num_els < self.arr.shape[0]:
			self.num_els += 1
		self.arr[self.ind] = value
		self.ind = (self.ind + 1) % self.arr.shape[0]

	def mean(self):
		return np.mean(self.arr[:self.num_els])

	def median(self):
		return np.median(self.arr[:self.num_els])
