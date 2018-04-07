#!/usr/bin/env python

import numpy as np

import rospy

import pickle

from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from ackermann_msgs.msg import AckermannDriveStamped
from final_challenge.msg import ConeInfo
from visualization_msgs.msg import Marker

from FinalChallengePy.Utils.GeomUtils import GeomUtils
from FinalChallengePy.Utils.LocalGlobalUtils import LocalGlobalUtils
from FinalChallengePy.Utils.MapUtils import MapUtils
from FinalChallengePy.Utils.RobotState import RobotState
from FinalChallengePy.Utils.VisualizeLine import VisualizeLine

from FinalChallengePy.PathPlanning.RRT import RRT
from FinalChallengePy.PathPlanning.FakeWall import FakeWall

from FinalChallengePy.RSS.Constants import *
from FinalChallengePy.TrajectoryTracking.TrajectoryTracker import TrajectoryTracker
from FinalChallengePy.TrajectoryTracking.Constants import *

from FinalChallengePy.Cone.Constants import RED_CONE_DIRECTION

from FinalChallengePy.Colors import *

class RSS(VisualizeLine):
    def __init__(self):
        VisualizeLine.__init__(self,"RSS",numPublishers=4)

        self.mapMsg = MapUtils.getMap()
        self.rangeLib = MapUtils.getRangeLib(self.mapMsg)
        self.rangeMethod = MapUtils.getRangeMethod(self.rangeLib, self.mapMsg)
        self.RRT = RRT(
                self.mapMsg, 
                maxIterations = RSS_MAX_ITERATIONS, 
                numOptimizations = RSS_NUM_OPTIMIZATIONS, 
                rangeMethod = self.rangeMethod,
                gaussianProbability = 0.8
                )

        self.state = RobotState(-1,0,3.14)

        self.drive = True

        # Load the points from file
        self.steers = pickle.load(open(BIG_PATH_FILE, 'rb'))
        path = RSS.steersToPath(self.steers)
        self.trajectoryTracker = TrajectoryTracker(path)

        self.fakeWalls = []

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

        rospy.loginfo("Almost done")

        for i in xrange(10):
            self.visualizePoints(path[0][0])
            rospy.sleep(0.1)

    def visualizeFakeWalls(self, fakeWalls):
        lineList = []
        for fakeWall in fakeWalls:
            lineList += fakeWall.getLine()
        self.visualize(lineList, WHITE, publisherIndex=2, lineList = True)

    def clickedPoint(self, msg):
        x = msg.point.x
        y = msg.point.y
        point = np.array([x, y])
        self.planAroundPoint(point, 1)

    def gotConeData(self, msg):
        x = msg.location.x
        y = msg.location.y
        pointLocal = np.array([x, y])
        pointGlobal = LocalGlobalUtils.localToGlobal(self.state, pointLocal)
        direction = msg.direction
        self.planAroundPoint(pointGlobal, direction)

    def goalPointVisualizer(self, points):
        self.visualize(points,GREY,publisherIndex=1,lineList=True)

    def gotLocalizationData(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        theta = MapUtils.quaternionToAngle(msg.pose.orientation)

        self.state = RobotState(x, y, theta)

        # Move state backwards
        self.state.lidarToRearAxle()

        if self.trajectoryTracker.outOfBounds:
            self.replanPath()

        if self.drive:
            self.trajectoryTracker.publishCommand(self.state, self.commandPub, self.goalPointVisualizer)

    def replanPath(self, badSteerIndex = 0):
        # If there is no intersection don't do anything!
        if badSteerIndex < 0:
            return

        rospy.loginfo("Replanning path")

        # Otherwise we must compute a new path
        # Stop driving!
        #self.drive = False

        goalStates = [self.steers[i].getGoalState() for i in xrange(badSteerIndex, len(self.steers))]

        bestGoal, tree = self.RRT.computePath(
                self.state, 
                goalStates, 
                fakeWalls = [f.getBufferedLine() for f in self.fakeWalls],
                sampleStates = [f.getSampleCenter() for f in self.fakeWalls],
                multipleGoals = True)

        self.visualize(self.RRT.treeToLineList(tree),RED,publisherIndex=3,lineList=True)

        if bestGoal < 0:
            rospy.loginfo("No path found")
            return

        rospy.loginfo("Path found!")

        newSteers = self.RRT.getSteers()

        # Replace the path
        self.steers[0:badSteerIndex+bestGoal+1] = newSteers

        # redo the trajectory tracker
        path = self.steersToPath(self.steers)
        self.trajectoryTracker = TrajectoryTracker(path)

        # Visualize the change
        self.visualizePoints(path[0][0])

        # Continue driving
        #self.drive = True

    '''
    recalculate the path to circle around
    point in the specified direction
    '''
    def planAroundPoint(self, point, direction):
        fakeWall = FakeWall(
                point, 
                self.state.getOrientation(), 
                direction, 
                self.rangeMethod
                )

        # update the fake walls

        # test to see if is already in the list
        isNew = True
        for index, f in enumerate(self.fakeWalls):
            if np.linalg.norm(f.point - fakeWall.point) < CONE_DISTANCE_THRESHOLD:
                # if it is, update it
                self.fakeWalls[index] = fakeWall
                isNew = False
                break

        # if it is not, add and pop!
        if isNew:
            self.fakeWalls.append(fakeWall)

        # clear ones that are behind the car
        # or very far away
        # or of negligible length
        for index, f in enumerate(self.fakeWalls):
            if LocalGlobalUtils.globalToLocal(self.state, f.point)[1] < 0 or\
                    np.linalg.norm(f.point - self.state.getPosition()) > CONE_VISIBILITY_THRESHOLD or \
                    f.distance < MIN_FAKE_WALL_DISTANCE:
                self.fakeWalls.pop(index)

        self.visualizeFakeWalls(self.fakeWalls)

        # Find the first section which intersects the fake wall path 
        badSteerIndex = -1
        badSteer = None
        for index, steer in enumerate(self.steers):
            if steer.intersects(fakeWall.getLine()):
                badSteerIndex = index
                badSteer = steer
                break

        self.replanPath(badSteerIndex)

    def visualizePoints(self, points):
        self.visualize(points,BLUE,publisherIndex=0)

    @staticmethod
    def steersToPath(steers):
        points = []
        for steer in steers:
            points += steer.getPoints()
        return [(points, False)]

if __name__=="__main__":
    rospy.init_node("RSS")
    rss = RSS()
    rospy.spin()
