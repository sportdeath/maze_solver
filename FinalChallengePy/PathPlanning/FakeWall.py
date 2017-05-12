from FinalChallengePy.Utils.GeomUtils import GeomUtils
from FinalChallengePy.PathPlanning.Constants import *

import numpy as np

class FakeWall:

    @staticmethod
    def makeFakeWall(point, normal, direction, rangeMethod):

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

        # The point on the wall
        endPoint = point + minDistance * minNorm

        point = point - FAKE_WALL_BUFFER * minNorm
        
        return (point, endPoint)
