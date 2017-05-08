import rospy
import numpy as np

from FinalChallengePy.Vision.CoordinateTransformations import CoordinateTransformations

from geometry_msgs.msg import Point

DISTANCE_THRESHOLD = 10 * 0.3048/1 # 10 ft in meters

class Cone:
    def __init__(self, conePixels):
        localPoint = CoordinateTransforms.pixelsToWorld(self.conePoint)
        self.position = LocalGlobalUtils.localToGlobal(localPoint)

    def getPosition(self):
        return self.position

    def isNewCone(self, cones):
        for cone in cones:
            distance = np.linalg.norm(self.getPosition() - cone.getPosition())
            if distance < self.DISTANCE_THRESHOLD:
                return False

        return True
