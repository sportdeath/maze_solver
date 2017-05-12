#!/usr/bin/env python

import numpy as np

import rospy
from geometry_msgs.msg import PoseStamped, PoseArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PointStamped

from FinalChallengePy.PathPlanning.FakeWall import FakeWall
from FinalChallengePy.PathPlanning.Steer import Steer 
from FinalChallengePy.PathPlanning.Sampling import Sampling

from FinalChallengePy.Utils.VisualizeLine import VisualizeLine
from FinalChallengePy.Utils.RobotState import RobotState
from FinalChallengePy.Utils.MapUtils import MapUtils

class Test_Sampling(VisualizeLine):
    def __init__(self):
        VisualizeLine.__init__(self,"Test", numPublishers = 3)

        self.mapMsg = MapUtils.getMap()
        self.rangeLib = MapUtils.getRangeLib(self.mapMsg)
        self.rangeMethod = MapUtils.getRangeMethod(self.rangeLib, self.mapMsg)

        self.point = np.array([-15.,.53])
        self.normal = np.array([0.,1.])

        self.direction = 1.

        self.clickSub = rospy.Subscriber(
                "/move_base_simple/goal", 
                PoseStamped, 
                self.clickedPose, 
                queue_size=1)

        self.clickSub = rospy.Subscriber(
                "/clicked_point", 
                PointStamped, 
                self.clickedPoint, 
                queue_size=1)

        self.samplePub  = rospy.Publisher(
                "/samples", 
                PoseArray, 
                queue_size = 1)

        for i in xrange(3):
            rospy.sleep(0.1)
            self.makeFakeWall()

    def clickedPoint(self, msg):
        x = msg.point.x
        y = msg.point.y
        self.point = np.array([x, y])

        self.makeFakeWall()

    def clickedPose(self, msg):
        # The received pose
        x = msg.pose.position.x
        y = msg.pose.position.y
        theta = MapUtils.quaternionToAngle(msg.pose.orientation)

        normalState = RobotState(x, y, theta)
        self.normal = normalState.getOrientation()

        self.makeFakeWall()

    def makeFakeWall(self):
        fakeWall = FakeWall(self.point, self.normal, self.direction, self.rangeMethod)
        linePoints = fakeWall.getLine()

        sampleState = fakeWall.getSampleCenter()
        states = [sampleState]

        for i in xrange(20):
            states.append(Sampling.getGaussianState(
                sampleState, 
                0.5, 
                0.5))

        self.visualize(linePoints, (1., 0., 0.), publisherIndex=0)
        MapUtils.publishStates(states, self.samplePub)

if __name__=="__main__":
    rospy.init_node("Test_Sampling")
    test_sampling = Test_Sampling()
    rospy.spin()
