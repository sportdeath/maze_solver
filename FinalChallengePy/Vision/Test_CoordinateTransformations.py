#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point

import numpy as np

from FinalChallengePy.Vision.CoordinateTransformations import CoordinateTransformations

class Test_CoordinateTransformations:
    def __init__(self):
        self.clickSub = rospy.Subscriber(
                "/zed/left/image_raw_color_mouse_left", 
                Point, 
                self.pointClicked, 
                queue_size=1)

    def pointClicked(self, msg):
        # Add the pixel value
        x = msg.x
        y = msg.y
        point = np.array([x, y])
        print("Clicked point:", point)
        local = CoordinateTransformations.pixelsToLocal(point)
        print("In local coordinates:", local)

if __name__=="__main__":
    rospy.init_node("Test_CoordinateTransformations")
    test = Test_CoordinateTransformations()
    rospy.spin()
