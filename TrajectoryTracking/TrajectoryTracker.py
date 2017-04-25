import numpy as np

from Config import *
from scipy import spatial

class TrajectoryTracker:

    def __init__(self, points):
        self.points = points
        self.neighborDistanceError = 0.1
        self.endPointDistanceError = 0.5

    def getGoalPointGlobal(self, location):
        bestPoints = []
        for point in self.points:
            distance = abs(np.linalg.norm(point - location) - LOOK_AHEAD_DISTANCE)
            if distance < self.neighborDistanceError:
                bestPoints.append(point)
        if bestPoints:
            return bestPoints[-1]
        else:
            return self.points[0]

    def atEndOfPath(self, location):
        print np.linalg.norm(location - self.points[-1])
        return np.linalg.norm(location - self.points[-1]) < self.endPointDistanceError
