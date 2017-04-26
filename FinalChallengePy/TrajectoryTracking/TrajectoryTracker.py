import numpy as np

from FinalChallengePy.TrajectoryTracking.PurePursuit import PurePursuit
from FinalChallengePy.CarConstants import *

LOOK_AHEAD_DISTANCE = 0.5
LOOK_AHEAD_DISTANCE_SQUARED = LOOK_AHEAD_DISTANCE * LOOK_AHEAD_DISTANCE
THRESHOLD = 0.2

class TrajectoryTracker:
    def __init__(self, points):
        self.points = points
        self.pointIndex = 0
        self.completed = False

    """
    returns (velocity, angle)
    """
    def getControlAngle(self, state, visualizeMethod=None):
        if self.completed:
            return (0,0)

        goalPointGlobal, self.pointIndex = PurePursuit.pickClosestPoint(
                state.getPosition(), 
                LOOK_AHEAD_DISTANCE_SQUARED,
                self.points,
                self.pointIndex)

        # If we are at the end!
        if self.pointIndex == len(self.points) - 1:
            if np.linalg.norm(goalPointGlobal - state.getPosition()) < THRESHOLD:
                self.completed = True
                return (0,0)

        goalPointLocal = PurePursuit.globalPointToLocal(state, goalPointGlobal)

        if visualizeMethod:
            TrajectoryTracker.visualize(state, goalPointGlobal, visualizeMethod)

        velocity = np.sign(goalPointLocal[1]) * CAR_VELOCITY
        angle = PurePursuit.getAckermannAngle(goalPointLocal)

        return (velocity, angle)


    @staticmethod
    def visualize(
            state,
            goalPointGlobal,
            visualizeMethod
            ):

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
