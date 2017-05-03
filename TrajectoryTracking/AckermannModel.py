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


class AckermannModel(object):
	""" A wrapper class for useful Ackermann steering geometry related functions
	"""
	def __init__(self, wheelbase):
		self.L = wheelbase

	def path_radius(self, steering_angle):
		''' The radius of the path driven if a constant steering angle is applied
		'''
		return self.L / np.tan(steering_angle)

	def yaw_rate(self, steering_angle, speed):
		''' Rate of change of heading with a given steering angle and speed
		'''
		if steering_angle == 0.0:
			return 0.0
		return speed / self.path_radius(steering_angle)

	def dx(self, speed, dt, steering_angle):
		''' Distance traveled in the local x direction given speed and steering_angle
		'''
		if steering_angle == 0.0:
			return speed * dt
		R = self.path_radius(steering_angle)
		d = dt*speed
		dx = R*np.sin(d/R)
		return dx

	def dy(self, speed, dt, steering_angle):
		''' Distance traveled in the local y direction given speed and steering_angle
		'''
		if steering_angle == 0.0:
			return 0.0
		R = self.path_radius(steering_angle)
		d = dt*speed
		dy = R*(1.0 - np.cos(d/R))
		return dy

	def steering_angle(self, point):
		''' Returns the steering angle required to pass through the given point
		    (in local euclidean coordinates) assuming constant steering angle is applied
		'''
		if point[0] >= 0.0:
			theta = np.arctan(point[1]/point[0])
		else:
			theta = np.arctan(abs(point[0])/point[1]) + np.sign(point[1])*np.pi/2.0

		return np.arctan(2.0*self.L*np.sin(theta)/np.linalg.norm(point))

	def steering_angle_polar(self, polar_point):
		''' Returns the steering angle required to pass through the given point
		    (in local polar coordinates) assuming constant steering angle is applied
		'''
		theta = polar_point[1]
		radius = polar_point[0]
		return np.arctan(2.0*self.L*np.sin(theta)/radius)
