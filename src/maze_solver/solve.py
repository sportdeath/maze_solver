#!/usr/bin/env python2

import numpy as np

import rospy
import tf
from rospy.numpy_msg import numpy_msg
from nav_msgs.msg import OccupancyGrid

from maze_solver.rrt.rrt_visualizer import RRTVisualizer
from maze_solver.rrt.rrt import RRT

class Solve:

    CARTOGRAPHER_DILATED_TOPIC = rospy.get_param("/maze_solver/cartographer_dilated_topic")
    CARTOGRAPHER_FRAME = rospy.get_param("/maze_solver/cartographer_frame")
    BASE_FRAME = rospy.get_param("/maze_solver/base_frame")
    NUM_ITERATIONS = rospy.get_param("/maze_solver/rrt_iterations")

    def __init__(self):
        # Get the pose
        self.tf_listener = tf.TransformListener()

        # Initialize path
        self.path = []

        # Subscribe to the dilated map
        self.map_sub = rospy.Subscriber(
                self.CARTOGRAPHER_DILATED_TOPIC,
                numpy_msg(OccupancyGrid),
                self.map_cb, 
                queue_size=1)

        # Make a visualizer
        self.visualizer = RRTVisualizer()

    def recompute_path(self, map_msg):
        while True:
            try:
                pose = self.tf_listener.lookupTransform(self.CARTOGRAPHER_FRAME, self.BASE_FRAME, rospy.Time(0))
                break
            except tf.ConnectivityException:
                continue

        pose = np.array([
            pose[0][0],
            pose[0][1],
            tf.transformations.euler_from_quaternion(pose[1])[2]])

        self.rrt = RRT(pose, map_msg)

        # Get the path from RRT
        while not self.path:
            print "Searching for path..."
            for i in xrange(self.NUM_ITERATIONS):
                self.rrt.iterate()
            self.path = self.rrt.path()
        print "Path found."

        self.visualizer.visualize_path(self.path)

        # Follow the trajectory
        # TODO

    def map_cb(self, map_msg):
        """
        Store and reshape the map. 

        Args:
            map_msg: A ROS numpy_msg(OccupancyGrid) message.
        """
        # Reshape
        map_msg.data.shape = (map_msg.info.height, map_msg.info.width)

        # Check if the goal path intersects
        recompute = (not self.path)
        for steer in self.path:
            if steer.intersects(self.rrt.sample_width, map_msg, self.rrt.OCCUPANCY_THRESHOLD):
                recompute = True
                break

        if recompute:
            self.path = []
            # Stop the trajectory tracker
            # TODO

            # Recompute
            self.recompute_path(map_msg)

if __name__ == "__main__":
    rospy.init_node("solve")
    Solve()
    rospy.spin()
