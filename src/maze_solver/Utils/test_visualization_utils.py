#!/usr/bin/env python2

"""
Draws an equation in rviz by publishing
a line to the topic "/line".
"""

import numpy as np

import rospy
from visualization_msgs.msg import Marker

from maze_solver.Utils.visualization_utils import VisualizationUtils

class TestVisualizationUtils:

    def __init__(self):
        # Make a marker publisher
        self.line_pub = rospy.Publisher(
                "/line", 
                Marker,
                queue_size=1)

        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.publish_test()
            r.sleep()

    def publish_test(self):
        # Make a quadratic function
        x = np.linspace(-3, 3, num=100)
        y = x * x

        # Plot it
        VisualizationUtils.plot(x, y, self.line_pub)

if __name__ == "__main__":
    rospy.init_node("test_visualization_utils")
    TestVisualizationUtils()
    rospy.spin()
