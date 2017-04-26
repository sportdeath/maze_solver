import numpy as np

class RobotState:
    def __init__(self, x, y, theta, backwards=False):
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

    def getOrientation(self):
        return self.orientation

    def getTheta(self):
        return self.theta

    def isBackwards(self):
        return self.backwards
