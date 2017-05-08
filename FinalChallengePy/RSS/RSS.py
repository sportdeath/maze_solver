#!/usr/bin/env python

import numpy as np

import rospy
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from ackermann_msgs.msg import AckermannDriveStamped
from final_challenge.msg import ConeInfo
from visualization_msgs.msg import Marker

from FinalChallengePy.Utils.GeomUtils import GeomUtils

from FinalChallengePy.PathPlanning.MapUtils import MapUtils
from FinalChallengePy.PathPlanning.RobotState import RobotState
from FinalChallengePy.PathPlanning.VisualizeLine import VisualizeLine
from FinalChallengePy.PathPlanning.RRT import RRT

from FinalChallengePy.RSS.Constants import *
from FinalChallengePy.TrajectoryTracking.TrajectoryTracker import TrajectoryTracker
from FinalChallengePy.TrajectoryTracking.Constants import *

from FinalChallengePy.Cone.Constants import RED_CONE_DIRECTION

class RSS(VisualizeLine):
    def __init__(self):
        VisualizeLine.__init__(self,"RSS",numPublishers=4)

        self.mapMsg = MapUtils.getMap()
        self.RRT = RRT(self.mapMsg)

        # Load the points from file
        self.points = list(np.loadtxt(BIG_PATH_FILE))
        self.trajectoryTracker = None

        self.commandPub = rospy.Publisher(
                "/vesc/high_level/ackermann_cmd_mux/input/nav_0",
                AckermannDriveStamped,
                queue_size = 1)

        self.clickSub = rospy.Subscriber(
                "/clicked_point", 
                PointStamped, 
                self.clickedPoint, 
                queue_size=1)

        self.poseSub = rospy.Subscriber(
                "/pf/viz/inferred_pose", 
                PoseStamped, 
                self.gotLocalizationData, 
                queue_size = 1)

        self.coneSub = rospy.Subscriber(
                "/cone_info", 
                ConeInfo, 
                self.gotConeData, 
                queue_size = 1)

        for i in xrange(2):
            self.conePublishers = rospy.Publisher(
                    "/cone_visualizer"+str(i),
                    Marker,
                    queue_size=1)

        for i in xrange(10):
            self.visualizePoints()
            rospy.sleep(0.1)
    
    def visualizeCones(self, cones, publisher, color):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "/base_link"

        cone = Marker()
        cone.type = Marker.CUBE
        cone.action = Marker.ADD
        cone.header = header
        cone.scale.x = 0.1
        cone.scale.y = 0.1
        cone.scale.z = 0.1
        cone.pose.orientation.w = 1.
        cone.color.a = 1.
        cone.color.r = color[0]
        cone.color.g = color[1]
        cone.color.b = color[2]

        for c in cones:
            p = Point()
            p.x = c.getPosition()[0]
            p.y = c.getPosition()[1]
            p.z = 0.0001
            cone.points.append(p)

        publisher.publish(cone)

    def gotConeData(self, msg):
        x = msg.location.x
        y = msg.location.y
        point = np.array([x, y])
        direction = msg.direction
        if direction == RED_CONE_DIRECTION:
            self.redCones.append(point)
        else:
            self.greenCones.append(point)
        self.visualizeCones(self.redCones, self.conePublishers[0], (1.,0.,0.))
        self.visualizeCones(self.greenCones, self.conePublishers[1], (0.,1.,0.))
        self.planAroundPoint(point, direction)

    def goalPointVisualizer(self, points):
        self.visualize(points,(0.,0.,1.),publisherIndex=1,lineList=True)

    def gotLocalizationData(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        theta = MapUtils.quaternionToAngle(msg.pose.orientation)

        self.state = RobotState(x, y, theta)

        # Move state backwards
        self.state.lidarToRearAxle()

        if self.trajectoryTracker:
            self.trajectoryTracker.publishCommand(self.state, self.commandPub, self.goalPointVisualizer)
        else:
            # determine start state
            goalPosition = self.points[0]
            goalOrientation = self.points[1] - self.points[0]
            goalOrientation /= np.linalg.norm(goalOrientation)
            goalState = RobotState(goalPosition, goalOrientation)

            # compute a path to it
            self.RRT.computePath(self.state, goalState, backwards=True)
            pathsToStart = self.RRT.getPaths(True, LOOK_AHEAD_DISTANCE) 
            paths = pathsToStart + [(self.points, False)]

            # Visualize
            (forwardPoints, backwardsPoints) = self.RRT.getLineLists()
            self.visualize(forwardPoints,(0.,1.,0.),publisherIndex=2,lineList=True)
            self.visualize(backwardsPoints,(1.,0.,0.),publisherIndex=3,lineList=True)
            
            # Go!
            self.trajectoryTracker = TrajectoryTracker(paths)

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

        self.trajectoryTracker.setPoints(self.points)

        # Visualize the change
        self.visualizePoints()

    def visualizePoints(self):
        self.visualize(self.points,(0.,1.,0.),publisherIndex=0)

if __name__=="__main__":
    rospy.init_node("RSS")
    rss = RSS()
    rospy.spin()
