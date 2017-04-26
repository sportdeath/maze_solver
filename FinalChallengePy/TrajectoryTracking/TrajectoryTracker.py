import numpy as np

from FinalChallengePy.TrajectoryTracking.PurePursuit import PurePursuit
from FinalChallengePy.CarConstants import *

from FinalChallengePy.TrajectoryTracking.Constants import *

class TrajectoryTracker:
    def __init__(self, paths):
        self.paths = paths # [(points, orientation)...]
        self.pointIndex = 0
        self.pathIndex = 0

    """
    returns (velocity, angle)
    """
    def getControlAngle(self, state, visualizeMethod=None):
        if self.pathIndex >= len(self.paths):
            return (0,0)

        path = self.paths[self.pathIndex]
        points = path[0]
        backwards = path[1]

        goalPointGlobal, self.pointIndex = PurePursuit.pickClosestPoint(
                state.getPosition(), 
                LOOK_AHEAD_DISTANCE_SQUARED,
                points,
                self.pointIndex)

        # If we are at the end!
        if self.pointIndex == len(points) - 1:
            if np.linalg.norm(goalPointGlobal - state.getPosition()) <= LOOK_AHEAD_DISTANCE:
                self.pathIndex += 1
                self.pointIndex = 0
                return (0,0)

        if visualizeMethod:
            TrajectoryTracker.visualize(state, goalPointGlobal, visualizeMethod)

        # If we are going backwards
        # we are driving from the goal to state
        #if backwards:
        #    state, goalPointGlobal = goalPointGlobal, state

        goalPointLocal = PurePursuit.globalPointToLocal(state, goalPointGlobal)

        velocity = CAR_VELOCITY
        if backwards:
            velocity *= -1.
        angle = PurePursuit.getAckermannAngle(goalPointLocal)

        return (velocity, angle)


    @staticmethod
    def visualize(state, goalPointGlobal, visualizeMethod):

        points = []

        # Visualize the goal point
        points += [state.getPosition(), goalPointGlobal]

        # Visualize a circle surrounding the states
        CIRCLE_SIZE = 20
        for i in xrange(CIRCLE_SIZE + 1):
            angle = 2 * np.pi * i/float(CIRCLE_SIZE)
            point = LOOK_AHEAD_DISTANCE * np.array([np.sin(angle), np.cos(angle)])
            point += state.getPosition()
            points.append(point)
            if i != 0 and i != CIRCLE_SIZE:
                points.append(point)

        # Visualize
        visualizeMethod(points)
