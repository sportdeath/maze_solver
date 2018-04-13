#!/usr/bin/env python2

import numpy as np

import rospy
import tf_conversions
from rospy.numpy_msg import numpy_msg
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray

from maze_solver.rrt.rrt_visualizer import RRTVisualizer
from maze_solver.rrt.rrt import RRT
from maze_solver.trajectory_tracking.trajectory_tracker import TrajectoryTracker

class Solve:

    CARTOGRAPHER_DILATED_TOPIC = rospy.get_param("/maze_solver/cartographer_dilated_topic")
    NUM_ITERATIONS = rospy.get_param("/maze_solver/rrt_iterations")

    def __init__(self):
        # Initialize path
        self.path = []

        # Make a visualizer
        self.visualizer = RRTVisualizer()

        # Trajectory Tracker
        self.tracker = TrajectoryTracker()


        # Subscribe to the dilated map
        self.map_sub = rospy.Subscriber(
                self.CARTOGRAPHER_DILATED_TOPIC,
                numpy_msg(OccupancyGrid),
                self.map_cb, 
                queue_size=1)

    def recompute_path(self, map_msg):
        if self.tracker.pose is None:
            print "Waiting for pose"
            return

        # Move the start slightly away from obstacles.

        print "Recomputing..."
        self.rrt = RRT(self.tracker.pose, map_msg)

        # Get the path from RRT
        self.path = []
        while (not self.path) and (not rospy.is_shutdown()):
            print "Searching for path..."
            for j in xrange(self.NUM_ITERATIONS):
                self.rrt.iterate()
            self.path = self.rrt.path()
            if len(self.rrt.nodes) == 1:
                print "Jittering..."
                pose = self.tracker.pose + np.random.normal(scale=0.05, size=3)
                self.rrt = RRT(pose, map_msg)

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

        # Check if the goal path intersects
        # recompute = (not self.path) or self.tracker.is_lost
        recompute = (not self.path)
        for steer, reverse in self.path:
            if steer.intersects(self.rrt.sample_width, self.map_msg, self.rrt.OCCUPANCY_THRESHOLD, reverse):
                recompute = True
                break

        if recompute:
            # Stop the trajectory tracker
            self.tracker.stop()

            # Recompute
            self.recompute_path(self.map_msg)

if __name__ == "__main__":
    rospy.init_node("solve")
    Solve()
    rospy.spin()
