#!/usr/bin/env python

import numpy as np

import rospy
from geometry_msgs.msg import PoseStamped, PoseArray

from FinalChallengePy.PathPlanning.Steer import Steer

from FinalChallengePy.Utils.VisualizeLine import VisualizeLine
from FinalChallengePy.Utils.RobotState import RobotState
from FinalChallengePy.Utils.MapUtils import MapUtils
from FinalChallengePy.Utils.GeomUtils import GeomUtils

class Test_Steer(VisualizeLine):
    def __init__(self):
        VisualizeLine.__init__(self,"Test")

        self.mapMsg = MapUtils.getMap()
        self.rangeLib = MapUtils.getRangeLib(self.mapMsg)
        self.rangeMethod = MapUtils.getRangeMethod(self.rangeLib, self.mapMsg)

        self.initialState = RobotState(-1,0,3.14)
        
        self.clickSub = rospy.Subscriber(
                "/move_base_simple/goal", 
                PoseStamped, 
                self.clickedPose, 
                queue_size=1)

        self.samplePub  = rospy.Publisher(
                "/samples", 
                PoseArray, 
                queue_size = 1)

        SIZE = 1000
        origin = self.initialState.getPosition()
        points = []
        for i in xrange(SIZE):
            angle = i * 2. * np.pi/float(SIZE)
            norm = GeomUtils.getVector(angle)
            dist = self.rangeMethod(origin, angle)
            point = origin + norm * dist
            points.append(point)

        for i in xrange(10):
            self.visualize(points,(1.,0.,0.))
            rospy.sleep(0.1)

    def clickedPose(self, msg):
        # The received pose
        x = msg.pose.position.x
        y = msg.pose.position.y
        theta = MapUtils.quaternionToAngle(msg.pose.orientation)

        finalState = RobotState(x, y, theta)

        steer = Steer(self.initialState, finalState, self.rangeMethod)

        if steer.exists:
            pointsAndNorms = steer.getPointsAndNorms()

            states = [RobotState(point[0][0], point[0][1], GeomUtils.getAngle(point[1])) for point in pointsAndNorms]

            if steer.isSteerable():
                color = (0.,1.,0.)
            else:
                color = (1.,0.,0.)
            self.visualize(steer.getPoints(),color)

            MapUtils.publishStates(states, self.samplePub)

if __name__=="__main__":
    rospy.init_node("Test_RRT")
    test_steer = Test_Steer()
    rospy.spin()
