#!/usr/bin/env python2

import numpy as np

import rospy
import tf_conversions
from rospy.numpy_msg import numpy_msg
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped

from maze_solver.rrt.rrt_visualizer import RRTVisualizer
from maze_solver.rrt.rrt import RRT
from maze_solver.trajectory_tracking.trajectory_tracker import TrajectoryTracker

class Track:

    MAP_DILATED_TOPIC = rospy.get_param("/maze_solver/cartographer_dilated_topic")
    CARTOGRAPHER_TOPIC = rospy.get_param("/maze_solver/cartographer_topic")
    NUM_ITERATIONS = rospy.get_param("/maze_solver/rrt_iterations")
    CLICK_GOAL_TOPIC = rospy.get_param("/maze_solver/click_goal_topic")

    def __init__(self):
        # Initialize path
        self.path = []

        # Make a visualizer
        self.visualizer = RRTVisualizer()

        # Trajectory Tracker
        self.tracker = TrajectoryTracker()

        # Subscribe to clicked points
        self.goal_sub = rospy.Subscriber(
                self.CLICK_GOAL_TOPIC,
                PoseStamped,
                self.click_cb,
                queue_size=1)

        # Subscribe to the dilated map
        self.map_msg = None
        self.map_dilated_sub = rospy.Subscriber(
                self.MAP_DILATED_TOPIC,
                numpy_msg(OccupancyGrid),
                self.map_cb, 
                queue_size=1)
        # Subscribe to the map
        self.map_sub = rospy.Subscriber(
                self.CARTOGRAPHER_TOPIC,
                numpy_msg(OccupancyGrid),
                lambda x: 0,
                queue_size=1)

    def compute_path(self, map_msg, goal):
        if self.tracker.pose is None:
            print "Waiting for pose"

        print "Recomputing..."
        self.rrt = RRT(self.tracker.pose, map_msg, goal=goal)

        # Get the path from RRT
        self.path = []
        while not self.path:
            print "Searching for path..."
            for i in xrange(self.NUM_ITERATIONS):
                self.rrt.iterate()
            self.path = self.rrt.path()
        print "Path found."

        self.visualizer.visualize_path(self.path)

        # Follow the trajectory
        self.tracker.track(self.path)

    def map_cb(self, map_msg):
        """
        Store and reshape the map. 

        Args:
            map_msg: A ROS numpy_msg(OccupancyGrid) message.
        """
        # Reshape
        map_msg.data.shape = (map_msg.info.height, map_msg.info.width)
        self.map_msg = map_msg

    def click_cb(self, clicked_pose_msg):
        if self.map_msg is None:
            print "No map yet"
            return
        clicked_pose = np.array([
            clicked_pose_msg.pose.position.x,
            clicked_pose_msg.pose.position.y,
            tf_conversions.transformations.euler_from_quaternion([
                clicked_pose_msg.pose.orientation.x,
                clicked_pose_msg.pose.orientation.y,
                clicked_pose_msg.pose.orientation.z,
                clicked_pose_msg.pose.orientation.w])[2]])
        self.tracker.stop()
        self.compute_path(self.map_msg, goal=clicked_pose)

if __name__ == "__main__":
    rospy.init_node("track")
    Track()
    rospy.spin()
