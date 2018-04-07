import numpy as np

from FinalChallengePy.TrajectoryTracking.PurePursuit import PurePursuit
from FinalChallengePy.CarConstants import *

from FinalChallengePy.TrajectoryTracking.Constants import *
from FinalChallengePy.Utils.LocalGlobalUtils import LocalGlobalUtils

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Header

class TrajectoryTracker:
    def __init__(self, paths):
        self.paths = paths # [(points, orientation)...]
        self.pointIndex = 0
        self.pathIndex = 0
        self.outOfBounds = False

    """
    returns (velocity, angle)
    """
    def getControlAngle(self, state, visualizeMethod=None):
        if self.pathIndex >= len(self.paths):
            return (0,0)

        path = self.paths[self.pathIndex]
        points = path[0]
        backwards = path[1]

        goalPointGlobal, self.pointIndex, outOfBounds = PurePursuit.pickClosestPoint(
                state.getPosition(), 
                LOOK_AHEAD_DISTANCE_SQUARED,
                points,
                self.pointIndex)

        self.outOfBounds = outOfBounds

        # If we are at the end!
        if np.linalg.norm(points[-1] - state.getPosition()) <= LOOK_AHEAD_DISTANCE + LOOK_AHEAD_BUFFER:
            self.pathIndex += 1
            self.pointIndex = 0
            return (0,0)

        if visualizeMethod:
            TrajectoryTracker.visualize(state, goalPointGlobal, visualizeMethod)

        goalPointLocal = LocalGlobalUtils.globalToLocal(state, goalPointGlobal)

        velocity = CAR_VELOCITY
        if len(points) - self.pointIndex < NUM_TUNE_POINTS:
            velocity = CAR_TUNE_VELOCITY

        if backwards:
            velocity *= -1.
        angle = PurePursuit.getAckermannAngle(goalPointLocal)

        np.clip(angle, -MAX_STEERING_ANGLE, MAX_STEERING_ANGLE)

        return (velocity, angle)

    def isPathComplete(self):
        return self.pathIndex >= len(self.paths)

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

    def publishCommand(self, state, publisher, visualizer = None):
        velocity, angle = self.getControlAngle(
                state,
                visualizer)

        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "pure_pursuit"

        drivingCommand = AckermannDriveStamped()
        drivingCommand.drive.speed = velocity
        drivingCommand.drive.steering_angle = angle

        publisher.publish(drivingCommand)