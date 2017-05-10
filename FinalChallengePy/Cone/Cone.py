import rospy
import numpy as np

from FinalChallengePy.Vision.CoordinateTransformations import CoordinateTransformations

from FinalChallengePy.Utils.LocalGlobalUtils import LocalGlobalUtils

from geometry_msgs.msg import Point

from FinalChallengePy.Cone.Constants import *

class Cone:
    def __init__(self, conePixels):
        self.position = CoordinateTransformations.pixelsToLocal(conePixels)

    def getPosition(self):
        return self.position

    def isNewCone(self, cones):
        for cone in cones:
            distance = np.linalg.norm(self.getPosition() - cone.getPosition())
            if distance < DISTANCE_THRESHOLD:
                return False

        return True
