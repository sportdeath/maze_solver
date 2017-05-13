#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from ackermann_msgs.msg import AckermannDriveStamped

from FinalChallengePy.PathPlanning.MapUtils import MapUtils
from FinalChallengePy.PathPlanning.RobotState import RobotState
from FinalChallengePy.PathPlanning.VisualizeLine import VisualizeLine
from FinalChallengePy.PathPlanning.RRT import RRT
from FinalChallengePy.TrajectoryTracking.TrajectoryTracker import TrajectoryTracker
from FinalChallengePy.TrajectoryTracking.Constants import *

class Parking(VisualizeLine):
    def __init__(self):
        VisualizeLine.__init__(self,"Parking",numPublishers=3)

        self.mapMsg = MapUtils.getMap()
        self.RRT = RRT(
                self.mapMsg,
                maxIterations = 300,
                numOptimizations = 0, 
                verbose = False, 
                gaussianProbability = 0.9,
                positionStdDev = 0.1,
                angleStdDev = 0.1)

        self.state = RobotState(0,0,0)

        self.trajectoryTracker = None

        self.parkedState = RobotState(-1.62, -3.5, 0)
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

        self.park()
        self.unpark()

    def goToState(self, goal):

        rospy.loginfo("Planning path from " + str(self.state.getPosition()) + " to " + str(goal.getPosition()))

        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            bestGoalIndex, tree = self.RRT.computePath(
                    self.state, 
                    goal, 
                    backwards=True,
                    sampleStates = [goal])

            # Visualize the tree
            self.visualize(self.RRT.treeToLineList(tree),(0.,0.,1.),publisherIndex=2,lineList=True)

            if bestGoalIndex > 0:
                (forwardPoints, backwardsPoints) = self.RRT.getLineLists()
                self.visualize(forwardPoints,(0.,1.,0.),publisherIndex=0,lineList=True)
                self.visualize(backwardsPoints,(1.,0.,0.),publisherIndex=1,lineList=True)

                # follow the path to that tree
                self.trajectoryTracker = TrajectoryTracker(self.RRT.getPoints(self.RRT.getPath(),True,LOOK_AHEAD_DISTANCE))

                rospy.loginfo("Path planning complete")
                break

            rospy.loginfo("Path not found :(")
            r.sleep()
        
        while not rospy.is_shutdown() or not self.trajectoryTracker.isPathComplete():
            r.sleep()

        self.complete = False

    def park(self):
        rospy.loginfo("Parking...")

        self.goToState(self.parkedState)

        rospy.loginfo("Finished parking")

    def unpark(self):
        rospy.loginfo("Unparking...")

        self.goToState(self.unparkedState)

        rospy.loginfo("Finished unparking")

    def goalPointVisualizer(self, points):
        self.visualize(points,(0.,0.,1.),publisherIndex=2,lineList=True)

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