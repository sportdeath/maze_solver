import numpy as np

class RobotState:
    def __init__(self, x, y, theta):
        self.position = np.array([x, y])
        self.theta = theta
        # TODO is this vector oriented right???
        self.orientation = np.array([np.sin(theta), np.cos(theta)])

    def __init__(self, position, theta):
        self.position = position
        self.theta = theta
        self.oreitnation = np.array([np.sin(theta), np.cos(theta)])
    
    def getPosition(self):
        return self.position

    def getOrientation(self):
        return self.orientation

    def getTheta(self):
        return self.theta

