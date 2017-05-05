#!/usr/bin/env python

import numpy as np

import rospy
from geometry_msgs.msg import PointStamped

from FinalChallengePy.Utils.GeomUtils import GeomUtils

from FinalChallengePy.PathPlanning.MapUtils import MapUtils
from FinalChallengePy.PathPlanning.RobotState import RobotState
from FinalChallengePy.PathPlanning.VisualizeLine import VisualizeLine
from FinalChallengePy.PathPlanning.RRT import RRT

from FinalChallengePy.RSS.Constants import *

class RSS(VisualizeLine):
    def __init__(self):
        VisualizeLine.__init__(self,"RSS",numPublishers=3)

        self.mapMsg = MapUtils.getMap()
        self.RRT = RRT(self.mapMsg)

        self.clickSub = rospy.Subscriber(
                "/clicked_point", 
                PointStamped, 
                self.clickedPoint, 
                queue_size=1)

        # Load the points from file
        self.points = list(np.loadtxt(BIG_PATH_FILE))

        for i in xrange(10):
            self.visualizePoints()
            rospy.sleep(0.1)

    def clickedPoint(self, msg):
        x = msg.point.x
        y = msg.point.y
        point = np.array([x, y])
        self.planAroundPoint(point, -1)

    '''
    recalculate the path to circle around
    point in the specified direction
    '''
    def planAroundPoint(self, point, direction):
        # find closest point on path
        closestDistSquared = np.inf
        closestIndex = 0
        for i, p in enumerate(self.points):
            difference = p - point
            distSquared = np.dot(difference, difference)
            if distSquared < closestDistSquared:
                closestDistSquared = distSquared
                closestIndex = i

        if closestIndex < len(self.points) - 1:
            middleOrientation = self.points[closestIndex + 1] - self.points[closestIndex]
        else:
            middleOrientation = self.points[closestIndex] - self.points[closestIndex - 1]
        middleOrientation = middleOrientation/np.linalg.norm(middleOrientation)

        perpendicular = GeomUtils.getPerpendicular(middleOrientation)
        middlePosition = point + direction * RSS_OFFSET * perpendicular

        vizPoints = [self.points[closestIndex], middlePosition]
        self.visualize(vizPoints,(0.,0.,1.),publisherIndex=1)

        middleState = RobotState(middlePosition, middleOrientation)

        # Get the points we leave and return to the path
        initIndex = max(0, closestIndex - RSS_NUM_POINTS)
        goalIndex = min(len(self.points) - 1, closestIndex + RSS_NUM_POINTS)

        # Construct the initial state
        initPosition = self.points[initIndex]
        initOrientation = self.points[initIndex + 1] - initPosition
        initOrientation = initOrientation/np.linalg.norm(initOrientation)
        initState = RobotState(initPosition, initOrientation)

        # and the goal state
        goalPosition = self.points[goalIndex]
        goalOrientation = goalPosition - self.points[goalIndex - 1]
        goalOrientation = goalOrientation/np.linalg.norm(goalOrientation)
        goalState = RobotState(goalPosition, goalOrientation)

        # Calculate both paths
        self.RRT.computePath(initState, middleState)
        newPath = self.RRT.getPaths()[0]
        self.RRT.computePath(middleState, goalState)
        newPath += self.RRT.getPaths()[0]

        # insert the new path into points
        self.points[initIndex : goalIndex + 1] = newPath

        # Visualize the change
        self.visualizePoints()

    def visualizePoints(self):
        self.visualize(self.points,(0.,1.,0.),publisherIndex=0)

if __name__=="__main__":
    rospy.init_node("RSS")
    rss = RSS()
    rospy.spin()
