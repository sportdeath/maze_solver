import rospy
import numpy as np
from yaml import load
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseArray, PolygonStamped, Point32
from std_msgs.msg import Header, ColorRGBA

import json

import tf.transformations
import tf

from utils import Utils

EPSILON = 0.00000000001

class LineTrajectory(object):
	""" A class to wrap and work with piecewise linear trajectories. """
	def __init__(self, viz_namespace=None):
		self.points = []
		self.np_points = None
		self.distances = []
		self.speed_profile = []
		self.has_acceleration = False
		self.visualize = False
		self.viz_namespace = viz_namespace
		self.speed_interpolater = None

		if viz_namespace:
			self.visualize = True
			self.start_pub = rospy.Publisher(viz_namespace + "/start_point", Marker, queue_size = 1)
			self.traj_pub  = rospy.Publisher(viz_namespace + "/path", Marker, queue_size = 1)
			self.end_pub   = rospy.Publisher(viz_namespace + "/end_pose", Marker, queue_size = 1)
			self.speed_pub   = rospy.Publisher(viz_namespace + "/speeds", MarkerArray, queue_size = 1)

	# compute the distances along the path for all path segments beyond those already computed
	def update_distances(self):
		num_distances = len(self.distances)
		num_points = len(self.points)

		for i in xrange(num_distances,num_points):
			if i == 0:
				self.distances.append(0)
			else:
				p0 = self.points[i-1]
				p1 = self.points[i]
				delta = np.array([p0[0]-p1[0],p0[1]-p1[1]])
				self.distances.append(self.distances[i-1] + np.linalg.norm(delta))

	def distance_to_end(self, t):
		if not len(self.points) == len(self.distances):
			print "WARNING: Different number of distances and points, this should never happen! Expect incorrect results. See LineTrajectory class."
		dat = self.distance_along_trajectory(t)
		if dat == None:
			return None
		else:
			return self.distances[-1] - dat

	def distance_along_trajectory(self, t):
		# compute distance along path
		# ensure path boundaries are respected
		if t < 0 or t > len(self.points) - 1.0:
			return None
		i = int(t) # which segment
		t = t % 1.0 # how far along segment
		if t < EPSILON:
			return self.distances[i]
		else:
			return (1.0-t)*self.distances[i] + t*self.distances[i+1]

	def addPoint(self, point):
		print "adding point to trajectory:", point.x, point.y
		self.points.append((point.x, point.y))
		self.update_distances()
		self.mark_dirty()

	def clear(self):
		self.points = []
		self.distances = []
		self.speed_profile = []
		self.speed_interpolater = None
		self.mark_dirty()

	def empty(self):
		return len(self.points) == 0

	def save(self, path):
		print "Saving trajectory to:", path
		data = {}
		data["points"] = []
		for p in self.points:
			data["points"].append({"x": p[0], "y": p[1]})
		with open(path, 'w') as outfile:
			json.dump(data, outfile)

	def mark_dirty(self):
		self.has_acceleration = False

	def dirty(self):
		return not self.has_acceleration

	def load(self, path):
		print "Loading trajectory:", path
		self.points = np.loadtxt(path)
		# with open(path) as json_file:
		# 	json_data = json.load(json_file)
		# 	for p in json_data["points"]:
		# 		self.points.append((p["x"], p["y"]))
		self.update_distances()
		print "Loaded:", len(self.points), "points"
		self.mark_dirty()
		
	# put the points into a KD tree for faster nearest neighbors queries
	def make_np_array(self):
		self.np_points = np.array(self.points)
		self.np_distances = np.array(self.distances)
		self.has_acceleration = True

	# build a trajectory class instance from a trajectory message
	def fromPolygon(self, trajMsg):
		for p in trajMsg.points:
			self.points.append((p.x, p.y))
			if p.z >= 0:
				self.speed_profile.append(p.z)
		self.update_distances()
		self.mark_dirty()
		print "Loaded new trajectory with:", len(self.points), "points"

	def toPolygon(self):
		poly = PolygonStamped()
		poly.header = Utils.make_header("/map")
		use_speed_profile = len(self.speed_profile) == len(self.points)
		for i in xrange(len(self.points)):
			p = self.points[i]
			pt = Point32()
			pt.x = p[0]
			pt.y = p[1]
			if use_speed_profile:
				pt.z = self.speed_profile[i]
			else:
				pt.z = -1
			poly.polygon.points.append(pt)
		return poly

	def publish_start_point(self, duration=0.0, scale=0.1):
		should_publish = len(self.points) > 0
		if self.visualize and self.speed_pub.get_num_connections() > 0:
			print "Publishing speed profile"
			marker = Marker()
			marker.header = Utils.make_header("/map")
			marker.ns = self.viz_namespace + "/trajectory"
			marker.id = 0
			marker.type = 2 # sphere
			marker.lifetime = rospy.Duration.from_sec(duration)
			if should_publish:
				marker.action = 0
				marker.pose.position.x = self.points[0][0]
				marker.pose.position.y = self.points[0][1]
				marker.pose.orientation.w = 1.0
				marker.scale.x = 1.0
				marker.scale.y = 1.0
				marker.scale.z = 1.0
				marker.color.r = 0.0
				marker.color.g = 1.0
				marker.color.b = 0.0
				marker.color.a = 1.0
			else:
				# delete marker
				marker.action = 2

			self.start_pub.publish(marker)
		# elif self.start_pub.get_num_connections() == 0:
		# 	print "Not publishing start point, no subscribers"

	def publish_end_point(self, duration=0.0):
		should_publish = len(self.points) > 1
		if self.visualize and self.end_pub.get_num_connections() > 0:
			print "Publishing end point"
			marker = Marker()
			marker.header = Utils.make_header("/map")
			marker.ns = self.viz_namespace + "/trajectory"
			marker.id = 1
			marker.type = 2 # sphere
			marker.lifetime = rospy.Duration.from_sec(duration)
			if should_publish:
				marker.action = 0
				marker.pose.position.x = self.points[-1][0]
				marker.pose.position.y = self.points[-1][1]
				marker.pose.orientation.w = 1.0
				marker.scale.x = 1.0
				marker.scale.y = 1.0
				marker.scale.z = 1.0
				marker.color.r = 1.0
				marker.color.g = 0.0
				marker.color.b = 0.0
				marker.color.a = 1.0
			else:
				# delete marker
				marker.action = 2

			self.end_pub.publish(marker)
		# elif self.end_pub.get_num_connections() == 0:
		# 	print "Not publishing end point, no subscribers"

	def publish_trajectory(self, duration=0.0):
		should_publish = len(self.points) > 1
		if self.visualize and self.traj_pub.get_num_connections() > 0:
			print "Publishing trajectory"
			marker = Marker()
			marker.header = Utils.make_header("/map")
			marker.ns = self.viz_namespace + "/trajectory"
			marker.id = 2
			marker.type = 4 # line strip
			marker.lifetime = rospy.Duration.from_sec(duration)
			if should_publish:
				marker.action = 0
				marker.scale.x = 0.3
				marker.scale.y = 0.3
				marker.scale.z = 0.05
				marker.color.r = 1.0
				marker.color.g = 1.0
				marker.color.b = 1.0
				marker.color.a = 0.5
				for p in self.points:
					pt = Point32()
					pt.x = p[0]
					pt.y = p[1]
					pt.z = -0.1
					marker.points.append(pt)
			else:
				# delete
				marker.action = 2
			self.traj_pub.publish(marker)
		elif self.traj_pub.get_num_connections() == 0:
			print "Not publishing trajectory, no subscribers"

	def publish_speeds(self, duration=0.0, scale=0.7):
		should_publish = len(self.speed_profile) > 1
		if should_publish and self.visualize and self.speed_pub.get_num_connections() > 0:
			if self.dirty():
				self.make_np_array()
			markers = [Utils.marker_clear_all("/map")]
			normed_speeds = np.array(self.speed_profile) / np.max(self.speed_profile)
			last_speed = 0.0
			for i, speed in enumerate(normed_speeds):
				if speed >= last_speed * 0.99:
					color = ColorRGBA(0, 1, 0, 0.8)
				else:
					color = ColorRGBA(1, 0, 0, 0.8)
				last_speed = speed
				markers.append(marker_from_point_radius(self.np_points[i,:], np.power(speed, 0.8) * scale,
					index=i, linewidth=0.05, color=color, lifetime=duration))

			marker_array = MarkerArray(markers=markers)
			self.speed_pub.publish(marker_array)

	def speed_at_t(self, t):
		if self.speed_interpolater == None:
			self.xs = np.arange(len(self.speed_profile))
			self.ys = self.speed_profile
			self.speed_interpolater = lambda x: np.interp(x,  self.xs, self.ys)

		return self.speed_interpolater(t)

	def publish_viz(self, duration=0):
		if not self.visualize:
			print "Cannot visualize path, not initialized with visualization enabled"
			return

		self.publish_start_point(duration=duration)
		self.publish_trajectory(duration=duration)
		self.publish_end_point(duration=duration)
		self.publish_speeds(duration=duration)

	@staticmethod
	def nearest_point_on_trajectory(point, trajectory):
		'''
		Return the nearest point along the given piecewise linear trajectory.
		Same as nearest_point_on_line_segment, but vectorized. This method is quite fast, time constraints should
		not be an issue so long as trajectories are not insanely long. 
			Order of magnitude: trajectory length: 1000 --> 0.0002 second computation (5000fps)
		point: size 2 numpy array
		trajectory: Nx2 matrix of (x,y) trajectory waypoints
			- these must be unique. If they are not unique, a divide by 0 error will destroy the world
		'''
		diffs = trajectory[1:,:] - trajectory[:-1,:]
		l2s   = diffs[:,0]**2 + diffs[:,1]**2
		# this is equivalent to the elementwise dot product
		dots = np.sum((point - trajectory[:-1,:]) * diffs[:,:], axis=1)
		t = np.clip(dots / l2s, 0.0, 1.0)
		projections = trajectory[:-1,:] + (t*diffs.T).T
		dists = np.linalg.norm(point - projections,axis=1)
		min_dist_segment = np.argmin(dists)
		return projections[min_dist_segment], dists[min_dist_segment], t[min_dist_segment], min_dist_segment

	@staticmethod
	def first_point_on_trajectory_intersecting_circle(point, radius, trajectory, t=0.0, wrap=False):
		''' starts at beginning of trajectory, and find the first point one radius away from the given point along the trajectory.
		Assumes that the first segment passes within a single radius of the point
		http://codereview.stackexchange.com/questions/86421/line-segment-to-circle-collision-algorithm
		'''
		start_i = int(t)
		start_t = t % 1.0
		first_t = None
		first_i = None
		first_p = None
		for i in xrange(start_i, trajectory.shape[0]-1):
			start = trajectory[i,:]
			end = trajectory[i+1,:]
			V = end - start
			
			a = np.dot(V,V)
			b = 2.0*np.dot(V, start - point)
			c = np.dot(start, start) + np.dot(point,point) - 2.0*np.dot(start, point) - radius*radius
			discriminant = b*b-4*a*c

			if discriminant < 0:
				continue
			# 	print "NO INTERSECTION"
			# else:
			# if discriminant >= 0.0:
			discriminant = np.sqrt(discriminant)
			t1 = (-b - discriminant) / (2.0*a)
			t2 = (-b + discriminant) / (2.0*a)
			if i == start_i:
				if t1 >= 0.0 and t1 <= 1.0 and t1 >= start_t:
					first_t = t1
					first_i = i
					first_p = start + t1 * V
					break
				if t2 >= 0.0 and t2 <= 1.0 and t2 >= start_t:
					first_t = t2
					first_i = i
					first_p = start + t2 * V
					break
			elif t1 >= 0.0 and t1 <= 1.0:
				first_t = t1
				first_i = i
				first_p = start + t1 * V
				break
			elif t2 >= 0.0 and t2 <= 1.0:
				first_t = t2
				first_i = i
				first_p = start + t2 * V
				break
		# wrap around to the beginning of the trajectory if no intersection is found1
		if wrap and first_p == None:
			for i in xrange(start_i):
				start = trajectory[i,:]
				end = trajectory[i+1,:]
				V = end - start
				
				a = np.dot(V,V)
				b = 2.0*np.dot(V, start - point)
				c = np.dot(start, start) + np.dot(point,point) - 2.0*np.dot(start, point) - radius*radius
				discriminant = b*b-4*a*c

				if discriminant < 0:
					continue
				discriminant = np.sqrt(discriminant)
				t1 = (-b - discriminant) / (2.0*a)
				t2 = (-b + discriminant) / (2.0*a)
				if t1 >= 0.0 and t1 <= 1.0:
					first_t = t1
					first_i = i
					first_p = start + t1 * V
					break
				elif t2 >= 0.0 and t2 <= 1.0:
					first_t = t2
					first_i = i
					first_p = start + t2 * V
					break

		return first_p, first_i, first_t

		# print min_dist_segment, dists[min_dist_segment], projections[min_dist_segment]