#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PolygonStamped
import time, os
from LineTrajectory import LineTrajectory

class LoadTrajectory(object):
	""" Loads a trajectory from the file system and publishes it to a ROS topic.
	"""
	def __init__(self):
		self.path      = rospy.get_param("~trajectory")
		self.pub_topic = rospy.get_param("~trajectory_topic")

		# initialize and load the trajectory
		self.trajectory = LineTrajectory("/loaded_trajectory")
		self.trajectory.load(self.path)

		self.traj_pub = rospy.Publisher(self.pub_topic, PolygonStamped, queue_size=1)

		# need to wait a short period of time before publishing  the first message
		time.sleep(0.5)
		
		# visualize the loaded trajectory for 5 seconds
		self.trajectory.publish_viz(duration=3.0)

		# send the trajectory
		self.publish_trajectory()

	def publish_trajectory(self):
		print "Publishing trajectory to:", self.pub_topic
		self.traj_pub.publish(self.trajectory.toPolygon())

if __name__=="__main__":
	rospy.init_node("load_trajectory")

	pf = LoadTrajectory()
	rospy.spin()