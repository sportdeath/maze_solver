#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Header
from ackermann_msgs.msg import AckermannDriveStamped

import numpy as np

"""
Author: Ariel Anders
This program implements a simple safety node based on laser
scan data.

Edited by: Winter Guerra

# Single scan from a planar laser range-finder
# This is the ROS message structure

Header header
# stamp: The acquisition time of the first ray in the scan.
# frame_id: The laser is assumed to spin around the positive Z axis
# (counterclockwise, if Z is up) with the zero angle forward along the x axis

float32 angle_min # start angle of the scan [rad]
float32 angle_max # end angle of the scan [rad]
float32 angle_increment # angular distance between measurements [rad]

float32 time_increment # time between measurements [seconds] - if your scanner
# is moving, this will be used in interpolating position of 3d points
float32 scan_time # time between scans [seconds]

float32 range_min # minimum range value [m]
float32 range_max # maximum range value [m]

float32[] ranges # range data [m] (Note: values < range_min or > range_max should be discarded)
float32[] intensities # intensity data [device-specific units]. If your
# device does not provide intensities, please leave the array empty.

"""

class Safety():
	def __init__(self):

		# Init subscribers and publishers
		self.sub = rospy.Subscriber("/scan", LaserScan,\
				self.lidarCB, queue_size=1)
				
		self.pub = rospy.Publisher("/vesc/low_level/ackermann_cmd_mux/input/safety",\
				AckermannDriveStamped, queue_size =1 )

		self.max_distance = rospy.get_param("~max_distance")
				
		rospy.loginfo("Safety node initialized")


	def lidarCB(self, msg):
		'''
		This callback is called everytime the laserscanner sends us data.
		This is about 40hz. The message received is a laserscan message
		'''
		num_scans = len(msg.ranges)
		min_index = int((-np.pi/6 - msg.angle_min)/msg.angle_increment)
		max_index = int(min_index + np.pi/3/msg.angle_increment)
		total = 0
		num_points = 0 	
		
		for i in msg.ranges[min_index:max_index]:
			if ((msg.range_max > i)):
				total += i
				num_points += 1

		if (num_points != 0):
			something_in_front_of_robot = (total/float(num_points) < .5)
			#rospy.loginfo("average_range = %f, something_in_front_of_robot = %f",\
			 #total/float(num_points), something_in_front_of_robot)
		else:
			something_in_front_of_robot = 0
			#rospy.loginfo("no data")

		# If the autonomous stack is trying to run us into a wall, then stop.
		if ( something_in_front_of_robot ):
			stop_msg = AckermannDriveStamped()
            # stop_msg.speed = 0
            # stop_msg.steering_angle = 0
			rospy.loginfo('stopped for obstacle')
			self.pub.publish(stop_msg)
			
		

2
if __name__=="__main__":
	# Tell ROS that we're making a new node.
	rospy.init_node("Safety_Node")

	# Init the node
	Safety()
	# Don't let this script exit while ROS is still running
	rospy.spin()

