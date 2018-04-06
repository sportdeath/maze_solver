#!/usr/bin/env python

import numpy as np

import rospy
from geometry_msgs.msg import PoseStamped, PoseArray

from FinalChallengePy.PathPlanning.Steer import Steer
from FinalChallengePy.PathPlanning.Sampling import Sampling

from FinalChallengePy.Utils.VisualizeLine import VisualizeLine
from FinalChallengePy.Utils.RobotState import RobotState
from FinalChallengePy.Utils.MapUtils import MapUtils
from FinalChallengePy.Utils.GeomUtils import GeomUtils

class Test_MiniSteer(VisualizeLine):
    def __init__(self):
        VisualizeLine.__init__(self,"Test")

        self.mapMsg = MapUtils.getMap()
        self.rangeLib = MapUtils.getRangeLib(self.mapMsg)
        self.rangeMethod = MapUtils.getRangeMethod(self.rangeLib, self.mapMsg)

        self.initState = RobotState(-1.34, -3.48, 0)
        
        self.clickSub = rospy.Subscriber(
                "/move_base_simple/goal", 
                PoseStamped, 
                self.clickedPose, 
                queue_size=1)

        r = rospy.Rate(3)
        while not rospy.is_shutdown():
            self.generateMiniSteer()
            r.sleep()

    def clickedPose(self, msg):
        # The received pose
        x = msg.pose.position.x
        y = msg.pose.position.y
        theta = MapUtils.quaternionToAngle(msg.pose.orientation)

        self.initState = RobotState(x, y, theta)

    def generateMiniSteer(self):

        goalState = Sampling.getMiniState(
                self.initState,
                0.1, # angle std dev
                0.1, # length std dev
                backwards = True)

        steer = Steer(self.initState, goalState, self.rangeMethod)

        if steer.exists:
            rospy.loginfo("Exists!")

            if steer.isSteerable():
                color = (0.,1.,0.)
            else:
                color = (1.,0.,0.)

            self.visualize(steer.getPoints(), color)

        else:
            rospy.loginfo("Could not make steer!!")

if __name__=="__main__":
    rospy.init_node("Test_MiniSteer")
    test_ministeer = Test_MiniSteer()
    rospy.spin()
