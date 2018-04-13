#!/usr/bin/env python2

import numpy as np
import dubins

import rospy
from rospy.numpy_msg import numpy_msg
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid

from maze_solver.steer.map_differences import MapDifferences
from maze_solver.utils.visualization_utils import VisualizationUtils
from maze_solver.utils.geom_utils import GeomUtils
from maze_solver.steer.steer import DubinsSteer

class TestDubinsIntersection:

    def __init__(self):
        self.map_differences = MapDifferences()

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

        # Subscribe to the map
        self.map_sub = rospy.Subscriber(
                "/cartographer_map_dilated",
                numpy_msg(OccupancyGrid),
                self.map_cb, 
                queue_size=1)

    def map_cb(self, msg):
        msg.data.resize(msg.info.height, msg.info.width)
        self.map_msg = msg

    def pose_cb(self, msg):
        # Initial pose and constants
        q0 = (0., 0., 0.)
        turning_radius = 1.
        x = msg.pose.position.x
        y = msg.pose.position.y
        theta = GeomUtils.quaternion_to_angle(msg.pose.orientation)
        q1 = (x, y, theta)

        steer = DubinsSteer(q0, q1, turning_radius)

        if steer.intersects(self.map_msg):
            color = (1., 0., 0.)
        else:
            color = (0., 1., 0.)

        # Plot it
        points = steer.sample(0.1)
        x = points[:, 0]
        y = points[:, 1]
        VisualizationUtils.plot(x, y, self.line_pub, color=color, frame="cartographer_map")

if __name__ == "__main__":
    rospy.init_node("test_dubins_intersection")
    TestDubinsIntersection()
    rospy.spin()
