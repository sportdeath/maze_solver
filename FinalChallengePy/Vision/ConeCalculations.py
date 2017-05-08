import rospy
import numpy as np

from CoordinateTransformations import CoordinateTransformations

from geometry_msgs.msg import Point

class ConeCalculations:

    DISTANCE_THRESHOLD = 10 * 0.3048/1 # 10 ft in meters

    def __init__(self, conePoint):
        self.conePoint = conePoint

        # for coordinate transformations
        # TODO: fix this once class is made
        self.coordinateTransforms = CoordinateTransformations(SOME_INPUT)

    # TODO: fix coordinate transformation function
    def getWorldCoordinatesAsPoint(self):
        worldPoint = self.coordinateTransforms.transformPixelsToWorld(self.conePoint)

        return Point(worldPoint[0], worldPoint[1], 0)

    def isNewCone(self, coneLocList):
        newWorldPoint = self.getWorldCoordinatesAsPoint()

        for coneLoc in coneLocList:
            if np.linalg.norm(np.array([newWorldPoint.x, newWorldPoint.y]) - coneLoc) < self.DISTANCE_THRESHOLD:
                return False

        return True