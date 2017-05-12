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
        self.RRT = RRT(self.mapMsg)

        self.state = RobotState(0,0,0)

        self.trajectoryTracker = None

        self.parkingPoint = rospy.get_param("~parking_point")
        self.exitPoint = rospy.get_param("~exit_point")

        self.wayPoints = np.array([self.parkingPoint, self.exitPoint])
        self.segmentsComplete = np.zeros(self.wayPoints.shape[0], dtype=bool)
        self.currentSegmentIndex = 0

        self.poseSub = rospy.Subscriber(
                "/pf/viz/inferred_pose", 
                PoseStamped, 
                self.gotLocalizationData, 
                queue_size = 1)

        self.commandPub = rospy.Publisher(
                "/vesc/high_level/ackermann_cmd_mux/input/nav_0",
                AckermannDriveStamped,
                queue_size = 1)

        self.parking()

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

    def goToPoint(self, point):
        goalState = RobotState(float(point[0]), float(point[1]), float(point[2]))

        # Visualize the tree
        rospy.loginfo("Planning path from " + str(self.state.getPosition()) + " to " + str(goalState.getPosition()))
        tree = self.RRT.computePath(self.state, goalState, backwards=True)
        (forwardPoints, backwardsPoints) = self.RRT.getLineLists()
        self.visualize(forwardPoints,(0.,1.,0.),publisherIndex=0,lineList=True)
        self.visualize(backwardsPoints,(1.,0.,0.),publisherIndex=1,lineList=True)
        rospy.loginfo("Path planning complete")

        # follow the path to that tree
        self.trajectoryTracker = TrajectoryTracker(self.RRT.getPoints(self.RRT.getPath(),True,LOOK_AHEAD_DISTANCE))
        self.segmentsComplete[self.currentSegmentIndex] = self.TrajectoryTracker.isPathComplete()

    def parking(self):
        rospy.loginfo("Parking...")

        if self.segmentsComplete[self.currentSegmentIndex]:
            self.currentSegmentIndex += 1

        if self.currentSegmentIndex >= len(self.segmentsComplete):
            rospy.loginfo("Finished parking challenge")
            return

        self.goToPoint(self.wayPoints[self.currentSegmentIndex])

    def clickedPose(self, msg):
        # The received pose
        x = msg.pose.position.x
        y = msg.pose.position.y
        theta = MapUtils.quaternionToAngle(msg.pose.orientation)

        goalState = RobotState(x, y, theta)

        # Visualize the tree
        rospy.loginfo("Planning path from " + str(self.state.getPosition()) + " to " + str(goalState.getPosition()))
        tree = self.RRT.computePath(self.state, goalState, backwards=True)
        (forwardPoints, backwardsPoints) = self.RRT.getLineLists()
        self.visualize(forwardPoints,(0.,1.,0.),publisherIndex=0,lineList=True)
        self.visualize(backwardsPoints,(1.,0.,0.),publisherIndex=1,lineList=True)
        rospy.loginfo("Path planning complete")

        # follow the path to that tree
        self.trajectoryTracker = TrajectoryTracker(self.RRT.getPoints(self.RRT.getPath(),True,LOOK_AHEAD_DISTANCE))

if __name__=="__main__":
    rospy.init_node("Parking")
    parking = Parking()
    rospy.spin()
