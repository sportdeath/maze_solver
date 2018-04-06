#!/usr/bin/env python

import numpy as np

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PointStamped

from FinalChallengePy.PathPlanning.FakeWall import FakeWall
from FinalChallengePy.PathPlanning.Steer import Steer 

from FinalChallengePy.Utils.MapUtils import MapUtils
from FinalChallengePy.Utils.VisualizeLine import VisualizeLine
from FinalChallengePy.Utils.RobotState import RobotState

class Test_FakeWall(VisualizeLine):
    def __init__(self):
        VisualizeLine.__init__(self,"Test", numPublishers = 3)

        self.mapMsg = MapUtils.getMap()
        self.rangeLib = MapUtils.getRangeLib(self.mapMsg)
        self.rangeMethod = MapUtils.getRangeMethod(self.rangeLib, self.mapMsg)

        self.point = np.array([-15.,.53])
        self.normal = np.array([0.,1.])

        self.direction = 1.

        self.circleCenter = np.array([-11.9,0.])
        self.radius = 1.

        self.clickSub = rospy.Subscriber(
                "/move_base_simple/goal", 
                PoseStamped, 
                self.clickedPose, 
                queue_size=1)
        
        self.clickSub = rospy.Subscriber(
                "/initialpose", 
                PoseWithCovarianceStamped, 
                self.clickedCircle, 
                queue_size=1)

        self.clickSub = rospy.Subscriber(
                "/clicked_point", 
                PointStamped, 
                self.clickedPoint, 
                queue_size=1)

        for i in xrange(3):
            rospy.sleep(0.1)
            self.makeFakeWall()

    def clickedCircle(self, msg):
        # The received pose
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        self.circleCenter = np.array([x, y])

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

        intersectionPoints = Steer.intersectCirclePoints(linePoints, self.circleCenter, self.radius)

        intersectionLineList = []
        for intersectionPoint in intersectionPoints:
            intersectionLineList.append(self.circleCenter)
            intersectionLineList.append(intersectionPoint)

        numCirclePoints = 100
        circlePoints = []
        for i in xrange(numCirclePoints):
            angle = i*2*np.pi/float(numCirclePoints)
            point = self.circleCenter + self.radius * np.array([np.cos(angle), np.sin(angle)])
            circlePoints.append(point)

        self.visualize(linePoints, (1., 0., 0.), publisherIndex=0)
        self.visualize(circlePoints, (0., 0., 1.), publisherIndex=1)
        self.visualize(intersectionLineList, (0., 1., 0.), publisherIndex=2, lineList=True)

if __name__=="__main__":
    rospy.init_node("Test_FakeWall")
    test_fakewall = Test_FakeWall()
    rospy.spin()
