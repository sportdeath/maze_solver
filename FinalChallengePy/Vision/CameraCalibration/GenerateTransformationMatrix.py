#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point

import numpy as np

from FinalChallengePy.Vision.CameraCalibration.GetTransformation import getTransformation
from FinalChallengePy.Vision.Constants import *

class TransformationGenerator:
    def __init__(self):
        # Real world values
        self.sources = [np.array([0.,0.6,1.]), np.array([0.,5.,1.])]

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
        if len(self.pixelCoords) >= len(self.sources):
            self.generateTransformation()

    def generateTransformation(self):
        cameraMatrixInv = np.loadtxt(CAMERA_MATRIX_INV_FILE)

        print("Generating transformations")
        destinations = [0]*len(self.sources)
        for i in xrange(len(self.sources)):
            destinations[i] = np.dot(cameraMatrixInv, self.pixelCoords[i])

        transformationMatrix = getTransformation(self.sources, destinations)

        # Write to file
        np.savetxt(TRANSFORMATION_MATRIX_FILE, transformationMatrix)
        np.savetxt(TRANSFORMATION_MATRIX_INV_FILE, np.linalg.inv(transformationMatrix))
        print("Written to file")

        from FinalChallengePy.Vision.CoordinateTransformations import CoordinateTransformations

        for i in xrange(2):
            world = CoordinateTransformations.pixelsToLocal(self.pixelCoords[i])
            print("World: ", world, ", expected: ", self.sources[i])
            pixels = CoordinateTransformations.localToPixels(self.sources[i])
            print("Pixels: ", pixels, ", expected: ", self.pixelCoords[i])

if __name__=="__main__":
    rospy.init_node("GenerateTransformationMatrix")
    generator = TransformationGenerator()
    rospy.spin()
