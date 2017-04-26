#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Header

from FinalChallengePy.PathPlanning.MapUtils import MapUtils
from FinalChallengePy.PathPlanning.RobotState import RobotState
from FinalChallengePy.PathPlanning.VisualizeLine import VisualizeLine
from FinalChallengePy.PathPlanning.RRT import RRT

from FinalChallengePy.TrajectoryTracking.TrajectoryTracker import TrajectoryTracker
from FinalChallengePy.TrajectoryTracking.Constants import *

class Test_TrajectoryTracker(VisualizeLine):
    def __init__(self):
        VisualizeLine.__init__(self,"Test_TrajectoryTracker",numPublishers=3)

        self.mapMsg = MapUtils.getMap()
        self.RRT = RRT(self.mapMsg)

        self.state = RobotState(0,0,0)

        self.trajectoryTracker = None

        self.clickSub = rospy.Subscriber(
                "/move_base_simple/goal", 
                PoseStamped, 
                self.clickedPose, 
                queue_size=1)

        self.poseSub = rospy.Subscriber(
                "/pf/viz/inferred_pose", 
                PoseStamped, 
                self.gotLocalizationData, 
                queue_size = 1)

        self.commandPub = rospy.Publisher(
                "/vesc/high_level/ackermann_cmd_mux/input/nav_0",
                AckermannDriveStamped,
                queue_size = 1)

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
            velocity, angle = self.trajectoryTracker.getControlAngle(
                    self.state,
                    self.goalPointVisualizer)

            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = "pure_pursuit"

            drivingCommand = AckermannDriveStamped()
            drivingCommand.drive.speed = velocity
            drivingCommand.drive.steering_angle = angle

            self.commandPub.publish(drivingCommand)

    def clickedPose(self, msg):
        # The received pose
        x = msg.pose.position.x
        y = msg.pose.position.y
        theta = MapUtils.quaternionToAngle(msg.pose.orientation)

        goalState = RobotState(x, y, theta)

        # Visualize the tree
        print("Planning path from", self.state.getPosition(), "to", goalState.getPosition())
        tree = self.RRT.computePath(self.state, goalState, backwards=True)
        (forwardPoints, backwardsPoints) = self.RRT.getLineLists()
        self.visualize(forwardPoints,(0.,1.,0.),publisherIndex=0,lineList=True)
        self.visualize(backwardsPoints,(1.,0.,0.),publisherIndex=1,lineList=True)

        # follow the path to that tree
        self.trajectoryTracker = TrajectoryTracker(self.RRT.getPaths(True,LOOK_AHEAD_DISTANCE))

if __name__=="__main__":
    rospy.init_node("Test_TrajectoryTracker")
    test_trajectoryTracker = Test_TrajectoryTracker()
    rospy.spin()
