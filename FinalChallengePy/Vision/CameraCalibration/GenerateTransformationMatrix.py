#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point

import numpy as np

from FinalChallengePy.Vision.CameraCalibration.GetTransformation import getTransformation
from FinalChallengePy.Vision.Constants import *

class TransformationGenerator:
    def __init__(self):
        # Real world values
        self.sources = [np.array([0.,0.,1.]), np.array([1.,1.,1.])]

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
        point = np.array([x, y, 1.])
        print("Clicked point:", point)

        self.pixelCoords.append(point)

        # Generate the transformation
        if len(self.pixelCoords) >= 2:
            self.generateTransformation()

    def generateTransformation(self):
        cameraMatrixInv = np.loadtxt(CAMERA_MATRIX_INV_FILE)

        print("Generating transformations")
        destinations = [0]*2
        for i in xrange(2):
            destinations[i] = np.dot(cameraMatrixInv, self.pixelCoords[i])

        (scalingFactor, rotationMatrix, translationVector) = \
                getTransformation(self.sources, destinations)

        # Write them to file
        np.savetxt(SCALING_FACTOR_FILE, np.array([scalingFactor]))
        np.savetxt(TRANSLATION_VECTOR_FILE, translationVector)
        np.savetxt(ROTATION_MATRIX_FILE, rotationMatrix)
        np.savetxt(ROTATION_MATRIX_INV_FILE, np.linalg.inv(rotationMatrix))
        print("Written to file")

        from FinalChallengePy.Vision.CoordinateTransformations import CoordinateTransformations

        for i in xrange(2):
            world = CoordinateTransformations.pixelsToWorld(self.pixelCoords[i])
            print("World: ", world, ", expected: ", self.sources[i])
            pixels = CoordinateTransformations.worldToPixels(self.sources[i])
            print("Pixels: ", pixels, ", expected: ", self.pixelCoords[i])

if __name__=="__main__":
    rospy.init_node("GenerateTransformationMatrix")
    generator = TransformationGenerator()
    rospy.spin()
