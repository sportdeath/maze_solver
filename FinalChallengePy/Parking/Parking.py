#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from ackermann_msgs.msg import AckermannDriveStamped

from FinalChallengePy.Utils.MapUtils import MapUtils
from FinalChallengePy.Utils.RobotState import RobotState
from FinalChallengePy.Utils.VisualizeLine import VisualizeLine
from FinalChallengePy.PathPlanning.RRT import RRT
from FinalChallengePy.TrajectoryTracking.TrajectoryTracker import TrajectoryTracker
from FinalChallengePy.TrajectoryTracking.Constants import *

from FinalChallengePy.Colors import *

class Parking(VisualizeLine):
    def __init__(self):
        VisualizeLine.__init__(self,"Parking",numPublishers=3)

        self.mapMsg = MapUtils.getMap()
        self.RRT = RRT(
                self.mapMsg,
                maxIterations = 4000,
                numOptimizations = 0, 
                verbose = True, 
                miniSteerProbability = 0.9,
                miniSteerAngleStdDev = 0.1,
                miniSteerLengthStdDev = 0.1)

        self.state = RobotState(0,0,0)

        self.trajectoryTracker = None

        self.parkedState = RobotState(-1.34, -3.48, 0)
        self.unparkedState = RobotState(0,0,0)

        self.poseSub = rospy.Subscriber(
                "/pf/viz/inferred_pose", 
                PoseStamped, 
                self.gotLocalizationData, 
                queue_size = 1)

        self.commandPub = rospy.Publisher(
                "/vesc/high_level/ackermann_cmd_mux/input/nav_0",
                AckermannDriveStamped,
                queue_size = 1)

        # Wait for lidar messages
        rospy.sleep(2)

        self.park()
        self.unpark()

    def goToState(self, init, goal, planBackwards = False):

        rospy.loginfo("Planning path from " + str(init.getPosition()) + " to " + str(goal.getPosition()))

        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if planBackwards:
                init, goal = goal, init

            bestGoalIndex, tree = self.RRT.computePath(
                    init, 
                    goal, 
                    backwards=True)

            if planBackwards:
                self.RRT.reverse()

            # Visualize the tree
            self.visualize(self.RRT.treeToLineList(tree),WHITE,publisherIndex=2,lineList=True)

            if bestGoalIndex >= 0:
                (forwardPoints, backwardsPoints) = self.RRT.getLineLists()
                self.visualize(forwardPoints,BLUE,publisherIndex=0,lineList=True)
                self.visualize(backwardsPoints,RED,publisherIndex=1,lineList=True)

                # follow the path to that tree
                self.trajectoryTracker = TrajectoryTracker(self.RRT.getPoints(self.RRT.getPath(),True,LOOK_AHEAD_DISTANCE))

                rospy.loginfo("Path planning complete")
                break

            rospy.loginfo("Path not found :(")
            r.sleep()
        
        while not (rospy.is_shutdown() or self.trajectoryTracker.isPathComplete()):
            self.visualize(forwardPoints,BLUE,publisherIndex=0,lineList=True)
            self.visualize(backwardsPoints,RED,publisherIndex=1,lineList=True)
            r.sleep()

    def park(self):
        rospy.loginfo("Parking...")

        self.goToState(self.state, self.parkedState, planBackwards = True)

        rospy.loginfo("Finished parking")

    def unpark(self):
        rospy.loginfo("Unparking...")

        self.goToState(self.parkedState, self.unparkedState)

        rospy.loginfo("Finished unparking")

    def goalPointVisualizer(self, points):
        self.visualize(points,GREY,publisherIndex=2,lineList=True)

    def gotLocalizationData(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        theta = MapUtils.quaternionToAngle(msg.pose.orientation)

        self.state = RobotState(x, y, theta)

        # Move state backwards
        self.state.lidarToRearAxle()

        if self.trajectoryTracker:
            self.trajectoryTracker.publishCommand(self.state, self.commandPub, self.goalPointVisualizer)

if __name__=="__main__":
    rospy.init_node("Parking")
    parking = Parking()
    rospy.spin()
