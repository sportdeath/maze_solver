import numpy as np
from FinalChallengePy.Utils.GeomUtils import GeomUtils
from FinalChallengePy.CarConstants import *

class RobotState:
    def __init__(self, x, y, theta=None, backwards=False):
        if theta == None:
            self.position = x
            self.orientation = y
            self.theta = GeomUtils.getAngle(self.orientation)
        else:
            self.position = np.array([x, y])
            self.theta = theta
            self.orientation = np.array([np.cos(theta), np.sin(theta)])
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
