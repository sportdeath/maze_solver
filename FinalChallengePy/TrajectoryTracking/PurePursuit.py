import numpy as np

from FinalChallengePy.Utils.GeomUtils import GeomUtils

from FinalChallengePy.CarConstants import *

class PurePursuit:

    """
    Takes a robot state and
    """
    @staticmethod
    def globalPointToLocal(state, globalPoint):
        translatedPoint = globalPoint - state.getPosition()

        # rotate into the up frame of reference
        angle = np.arccos(state.getOrientation()[1]) \
                * np.sign(state.getOrientation()[0])

        return GeomUtils.rotateVector(translatedPoint, angle)

    """
    goal point is relative to the robot's
    frame of reference
    """
    @staticmethod
    def getAckermannAngle(goalPointLocal):
        goalDistance = np.linalg.norm(goalPointLocal)

        sinAlpha = goalPointLocal[0]/goalDistance
        curvature = 2*sinAlpha/goalDistance

        return -np.arctan(curvature * CAR_AXLE_DISTANCE)

    """
    Returns a point, interpolated from points
    that is closest to the look ahead distance
    and the index of that point.

    points are of the form [(position, backwards), ...]
    where backwards implies wee must drive to the point
    backwards

    return (point, index)
    """
    @staticmethod
    def pickClosestPoint(position, lookAheadDistSquared, points, prevIndex):
        point = None

        # start from the previous index and 
        # stop as soon as we get to a points
        prevDifference = points[prevIndex] - position

        prevDifferenceNormSquared = np.dot(prevDifference, prevDifference)

        if prevDifferenceNormSquared >= lookAheadDistSquared:
            return (points[prevIndex], prevIndex)

        for i in xrange(prevIndex + 1, len(points)):
            difference = points[i] - position

            if np.dot(difference, difference) >= lookAheadDistSquared:
                differenceBetweenPoints = points[i-1] - points[i]
                a = np.dot(differenceBetweenPoints, differenceBetweenPoints)
                b = 2 * np.dot(differenceBetweenPoints,difference)
                c = np.dot(difference,difference) - lookAheadDistSquared

                root = (- b - np.sqrt(b*b - 4 * a * c))/(2 * a)

                interpolated = points[i] + root * differenceBetweenPoints

                point = (interpolated, i - 1)
                break

        if not point:
            point = (points[-1], len(points) - 1)
        
        return point
