#!/usr/bin/env python

import numpy as np

import rospy
from geometry_msgs.msg import PoseStamped

from MapUtils import MapUtils
from RobotState import RobotState
from VisualizeLine import VisualizeLine
from Steer import Steer

class Test_Steer(VisualizeLine):
    def __init__(self):
        VisualizeLine.__init__(self,"Test_Steer")

        self.mapMsg = MapUtils.getMap()
        self.rangeLib = MapUtils.getRangeLib(self.mapMsg)
        self.rangeMethod = MapUtils.getRangeMethod(self.rangeLib, self.mapMsg)

        self.initialState = RobotState (-1.7,-3.5,0)
        
        self.clickSub = rospy.Subscriber(
                "/move_base_simple/goal", 
                PoseStamped, 
                self.clickedPose, 
                queue_size=1)

        SIZE = 1000
        origin = np.array([0.,0.])
        points = []
        for i in xrange(SIZE):
            angle = i * 2. * np.pi/float(SIZE)
            norm = np.array([np.sin(angle), np.cos(angle)])
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
            if steer.isSteerable():
                color = (0.,1.,0.)
            else:
                color = (1.,0.,0.)
            self.visualize(steer.getPoints(),color)

if __name__=="__main__":
    rospy.init_node("Test_RRT")
    test_steer = Test_Steer()
    rospy.spin()
