#!/usr/bin/env python2

import numpy as np
import dubins

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped

from maze_solver.utils.visualization_utils import VisualizationUtils
from maze_solver.utils.geom_utils import GeomUtils

class TestDubinsPath:

    def __init__(self):
        # Make a marker publisher
        self.line_pub = rospy.Publisher(
                "/line", 
                Marker,
                queue_size=1)

        self.goal_sub = rospy.Subscriber(
                "/move_base_simple/goal", 
                PoseStamped, 
                self.pose_cb, 
                queue_size=1)

    def pose_cb(self, msg):

        # Initial pose and constants
        q0 = (0., 0., 0.)
        turning_radius = 1.
        step_size = 0.1

        # End pose
        x = msg.pose.position.x
        y = msg.pose.position.y
        theta = GeomUtils.quaternion_to_angle(msg.pose.orientation)
        q1 = (x, y, theta)

        # Compute path
        path = dubins.shortest_path(q0, q1, turning_radius)
        points, _ = path.sample_many(step_size)

        # Convert to x, y
        points = np.array(points)
        x = points[:,0]
        y = points[:,1]

        # Plot it
        VisualizationUtils.plot(x, y, self.line_pub)

if __name__ == "__main__":
    rospy.init_node("test_dubins_path")
    TestDubinsPath()
    rospy.spin()
