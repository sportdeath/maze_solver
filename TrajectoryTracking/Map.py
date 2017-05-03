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

class Map(object):
	""" Convenience wrapper for an occupancy grid map object.
	    Provides methods to:
	        - check distances to nearest objects
	        - check if points are permissible
	        - mark map regions as explored
	        - dilate the map
	"""
	def __init__(self, map_msg):
		self.memoize_disks = True
		self.map_msg = map_msg
		self.map_info = map_msg.info
		#  # 0: permissible, -1: unmapped, 100: blocked
		self.raw_matrix = np.array(map_msg.data).reshape((map_msg.info.height, map_msg.info.width))
		self.exploration_coeff = float(rospy.get_param("~exploration_coeff", 0.75))

		# reversed from expectation since this is what distance_transform_edt requires
		self.occupancy_grid = np.ones_like(self.raw_matrix, dtype=bool)
		self.occupancy_grid[self.raw_matrix>50] = 0

		# 0: not permissible, 1: permissible
		self.permissible_region = np.zeros_like(self.raw_matrix, dtype=bool)
		self.permissible_region[self.raw_matrix==0] = 1

		self.distmap = ndimage.distance_transform_edt(self.occupancy_grid)
		self.exploration_buffer = np.zeros_like(self.raw_matrix, dtype=bool)

		if self.memoize_disks:
			self.memo_table = {}
			self.memoized = 0
			self.unmemoized = 0

	def get_permissible(self, queries, check_bounds=False, coord_convert=True):
		''' Given a Nx3 (x,y,theta) numpy array of queries, this returns the distances to the nearest obstacle at each query position.
		'''
		if coord_convert:
			q = queries.copy()
			world_to_map(q, self.map_info)
			q = np.round(q[:,:2]).astype(int)
		else:
			q = queries.astype(int)
		
		if check_bounds:
			bad = np.unique(np.concatenate((np.argwhere(q<0)[:,0], \
						   np.argwhere(q[:,1] >= self.occupancy_grid.shape[0])[:,0],  \
						   np.argwhere(q[:,0] >= self.occupancy_grid.shape[1])[:,0])))
			q[bad,:] = 0

		distances = self.permissible_region[q[:,1], q[:,0]]
		if check_bounds:
			distances[bad] = np.nan
		return distances

	def get_distances(self, queries, check_bounds=False, coord_convert=True):
		''' Given a Nx3 (x,y,theta) numpy array of queries, this returns the distances to the nearest obstacle at each query position.
		'''
		if coord_convert:
			q = queries.copy()
			world_to_map(q, self.map_info)
			q = np.round(q[:,:2]).astype(int)
		else:
			q = queries.astype(int)

		if check_bounds:
			bad = np.unique(np.concatenate((np.argwhere(q<0)[:,0], \
						   np.argwhere(q[:,1] >= self.occupancy_grid.shape[0])[:,0],  \
						   np.argwhere(q[:,0] >= self.occupancy_grid.shape[1])[:,0])))
			q[bad,:] = 0

		distances = self.distmap[q[:,1], q[:,0]] * self.map_info.resolution
		if check_bounds:
			distances[bad] = np.nan
		return distances

	def clear_exploration_buffer(self):
		self.exploration_buffer.fill(0)

	def add_circle_to_exploration_buffer(self, circle):
		''' marks a circular region of the exploration_buffer as explored
		'''
		position = np.array([circle.center.copy()])
		world_to_map(position, self.map_info)
		radius_pixels = int(np.ceil(circle.radius / self.map_info.resolution) * 0.75)
		x_center, y_center = int(position[0,1]), int(position[0,0])
		mask = skimage.morphology.disk(radius_pixels).astype(bool)
		half_size = int(mask.shape[0]/2.0)
		self.exploration_buffer[x_center-half_size:x_center+half_size+1, y_center-half_size:y_center+half_size+1] += mask

	def add_circles_to_exploration_buffer(self, poses, radii, exp_coeff=None):
		if exp_coeff == None:
			exp_coeff = self.exploration_coeff
		world_to_map(poses, self.map_info)
		radii_pixels = (np.ceil(radii / self.map_info.resolution) * exp_coeff).astype(int)

		for i in xrange(poses.shape[0]):
			x_center, y_center = int(poses[i,1]), int(poses[i,0])
			if self.memoize_disks and radii_pixels[i] in self.memo_table:
				mask = self.memo_table[radii_pixels[i]]
				self.memoized += 1
			else:
				mask = skimage.morphology.disk(radii_pixels[i]).astype(bool)
				if self.memoize_disks:
					self.unmemoized += 1
					self.memo_table[radii_pixels[i]] = mask
			half_size = int(mask.shape[0]/2.0)
			# print half_size
			self.exploration_buffer[x_center-half_size:x_center+half_size+1, y_center-half_size:y_center+half_size+1] += mask

	def get_explored(self, queries, check_bounds=False, coord_convert=True):
		''' Given a Nx3 (x,y,theta) numpy array of queries, this returns the distances to the nearest obstacle at each query position.
		'''
		if coord_convert:
			q = queries.copy()
			world_to_map(q, self.map_info)
			q = np.round(q[:,:2]).astype(int)
		else:
			q = queries.astype(int)
		
		if check_bounds:
			bad = np.unique(np.concatenate((np.argwhere(q<0)[:,0], \
						   np.argwhere(q[:,1] >= self.occupancy_grid.shape[0])[:,0],  \
						   np.argwhere(q[:,0] >= self.occupancy_grid.shape[1])[:,0])))
			q[bad,:] = 0

		explored = self.exploration_buffer[q[:,1], q[:,0]]
		if check_bounds:
			explored[bad] = np.nan
		return explored

	def get_exploration_occupancy_grid(self):
		msg = OccupancyGrid()
		msg.header = make_header("/map")
		msg.info = self.map_info
		buff = self.exploration_buffer.astype(np.int8)
		buff[buff == 0] = 100
		buff[buff == 1] = 50
		msg.data = buff.reshape(self.map_info.height*self.map_info.width).tolist()
		return msg

	def dilate(self, radius):
		og = np.zeros_like(self.occupancy_grid, dtype=float)
		og[self.occupancy_grid==0] = 255
		el = skimage.morphology.disk(10)
		return skimage.morphology.dilation(og, selem=el)
