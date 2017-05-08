#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point

import numpy as np

from FinalChallengePy.Vision.CameraCalibration.GetTransformation import getTransformation
from FinalChallengePy.Vision.Constants import *

class TransformationGenerator:
    def __init__(self):
        # Real world values
        self.sources = [np.array([0,0,0]), np.array([1,1,0])]

        # Get pixel values
        self.pixelCoords = []

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

        # Generate the transformation
        if len(pixelCoords) >= 2:
            self.generateTransformation()

    def generateTransformation(self):
        destinations = [0]*2
        for i in xrange(2):
            destinations[i] = dot(cameraMatrixInv, self.pixelCoords[i])

        (scalingFactor, rotationMatrix, translationVector) = \
                getTransformation(self.sources, destinations)

        # Write them to file
        np.savetxt(SCALING_FACTOR_FILE, scalingFactor)
        np.savetxt(TRANSLATION_VECTOR_FILE, translationVector)
        np.savetxt(ROTATION_MATRIX_FILE, rotationMatrix)
        np.savetxt(ROTATION_MATRIX_INV_FILE, np.linalg.inv(rotationMatrix))

if __name__=="__main__":
    rospy.init_node("GenerateTransformationMatrix")
    generator = TransformationGenerator()
    rospy.spin()
