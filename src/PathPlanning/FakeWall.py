from FinalChallengePy.Utils.GeomUtils import GeomUtils
from FinalChallengePy.PathPlanning.Constants import *

from FinalChallengePy.Utils.RobotState import RobotState

import numpy as np

class FakeWall:

    def __init__(self, point, normal, direction, rangeMethod):

        # Find the minimum angle (the closest wall)
        minDistance = np.inf
        minAngle = 0
        for i in xrange(NUM_WALL_ANGLE_SAMPLES):
            testAngle = i * 2.*np.pi/float(NUM_WALL_ANGLE_SAMPLES)
            distance = rangeMethod(point, testAngle)
            if distance < minDistance:
                minDistance = distance
                minAngle = testAngle

        minNorm = GeomUtils.getVector(minAngle)
        orientation = np.sign(np.cross(minNorm, normal))
        if orientation != direction:
            minAngle = minAngle + np.pi
            minNorm = -minNorm

        minDistance = rangeMethod(point, minAngle)

        self.angle = minAngle
        self.norm = minNorm
        self.distance = minDistance
        self.point = point
        self.endPoint = point + self.distance * self.norm
        self.direction = direction
        self.rangeMethod = rangeMethod
    
    def getBufferedLine(self):
        return [self.endPoint, self.point - FAKE_WALL_BUFFER*self.norm]
    
    def getLine(self):
        return [self.endPoint, self.point - FAKE_WALL_EXTENSION*self.norm]

    def getSampleCenter(self):
        # sample half way in between
        # buffered distance
        point = self.getLine()[1]
        distanceToWall = self.rangeMethod(point, self.angle+np.pi)
        center = point - distanceToWall/2. * self.norm

        # orientation = perpendicular to norm
        angle = self.angle + self.direction * np.pi/2.

        return RobotState(center[0], center[1], angle)
