import numpy as np

class RobotState:
    def __init__(self, x, y, theta):
        self.position = np.array([x, y])
        self.theta = theta
        self.orientation = np.array([np.cos(theta), np.sin(theta)])
    
    def getPosition(self):
        return self.position

    def getOrientation(self):
        return self.orientation

    def getTheta(self):
        return self.theta

