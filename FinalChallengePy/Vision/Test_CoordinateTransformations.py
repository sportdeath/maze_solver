#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point

import numpy as np
import cv2

from FinalChallengePy.Vision.CoordinateTransformations import CoordinateTransformations
from FinalChallengePy.Vision.Constants import *

class Test_CoordinateTransformations:
    def __init__(self):
        img = cv2.imread(TEST_IMAGE)

        NUM_STEPS = 200
        STEP_SIZE = 0.5

        for i in xrange(NUM_STEPS):
            print(i)
            for j in xrange(NUM_STEPS):
                x = (i - NUM_STEPS/2) * STEP_SIZE
                y = (j - NUM_STEPS/2) * STEP_SIZE
                point = np.array([x, y])
                pixels = CoordinateTransformations.localToPixels(point)
                pixels = (int(pixels[0]), int(pixels[1]))
                cv2.circle(img,pixels,2,[0,0,255],thickness=1)

        cv2.imwrite(TEST_IMAGE_OUT, img)

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
