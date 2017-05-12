import rospy
import numpy as np

from FinalChallengePy.Vision.CoordinateTransformations import CoordinateTransformations

from FinalChallengePy.Utils.LocalGlobalUtils import LocalGlobalUtils

from geometry_msgs.msg import Point

from FinalChallengePy.Cone.Constants import *

class Cone:
    def __init__(self, conePixels, direction):
        self.position = CoordinateTransformations.pixelsToLocal(conePixels)
        self.direction = direction

    def getPosition(self):
        return self.position

    def getDirection(self):
        return self.direction
