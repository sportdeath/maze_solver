import numpy as np
from FinalChallengePy.Utils.GeomUtils import GeomUtils
from FinalChallengePy.CarConstants import *

class RobotState:
    def __init__(self, x, y, theta, backwards=False):
        self.position = np.array([x, y])
        self.theta = theta
        self.orientation = GeomUtils.getVector(theta)
        self.backwards = backwards

    def getX(self):
        return self.position[0]

    def getY(self):
        return self.position[1]
    
    def getPosition(self):
        return self.position

    def lidarToRearAxle(self):
        self.position = self.position - DISTANCE_FROM_REAR_AXLE_TO_LIDAR * self.orientation

    def getOrientation(self):
        return self.orientation

    def getTheta(self):
        return self.theta

    def isBackwards(self):
        return self.backwards
