#!/usr/bin/env python

import numpy as np

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PointStamped

from FinalChallengePy.PathPlanning.FakeWall import FakeWall
from FinalChallengePy.PathPlanning.Steer import Steer 

from FinalChallengePy.Utils.VisualizeLine import VisualizeLine
from FinalChallengePy.Utils.RobotState import RobotState
from FinalChallengePy.Utils.MapUtils import MapUtils

class Test_FakeWall(VisualizeLine):
    def __init__(self):
        VisualizeLine.__init__(self,"Test", numPublishers = 3)

        self.mapMsg = MapUtils.getMap()
        self.rangeLib = MapUtils.getRangeLib(self.mapMsg)
        self.rangeMethod = MapUtils.getRangeMethod(self.rangeLib, self.mapMsg)

        self.point = np.array([-15.,.53])
        self.normal = np.array([-1.,0.])

        self.direction = 1.

        self.initState = RobotState(-4.4,0.,0)
        self.goalState = RobotState(-17.3,-1.,0)

        self.clickSub = rospy.Subscriber(
                "/move_base_simple/goal", 
                PoseStamped, 
                self.setGoal, 
                queue_size=1)
        
        self.clickSub = rospy.Subscriber(
                "/initialpose", 
                PoseWithCovarianceStamped, 
                self.setInitial, 
                queue_size=1)

        self.clickSub = rospy.Subscriber(
                "/clicked_point", 
                PointStamped, 
                self.setFakeWall, 
                queue_size=1)

        for i in xrange(3):
            rospy.sleep(0.1)
            self.makeFakeWall()

    def setInitial(self, msg):
        # The received pose
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        theta = MapUtils.quaternionToAngle(msg.pose.pose.orientation)

        self.initState = RobotState(x, y, theta)

        self.makeFakeWall()

    def setFakeWall(self, msg):
        x = msg.point.x
        y = msg.point.y
        self.point = np.array([x, y])

        self.makeFakeWall()

    def setGoal(self, msg):
        # The received pose
        x = msg.pose.position.x
        y = msg.pose.position.y
        theta = MapUtils.quaternionToAngle(msg.pose.orientation)

        self.goalState = RobotState(x, y, theta)

        self.makeFakeWall()

    def makeFakeWall(self):
        fakeWall = FakeWall(self.point, self.normal, self.direction, self.rangeMethod)
        linePoints = fakeWall.getLine()

        steer = Steer(self.initState, self.goalState, self.rangeMethod)

        if steer.isSteerable():
            intersects = steer.intersects(linePoints)

            if intersects:
                color = (1., 0., 0.) # red
            else:
                color = (0., 1., 0.) # green

            self.visualize(linePoints, (0., 0., 1.), publisherIndex=0)
            self.visualize(steer.getPoints(), color, publisherIndex=1)

if __name__=="__main__":
    rospy.init_node("Test_FakeWall")
    test_fakewall = Test_FakeWall()
    rospy.spin()
