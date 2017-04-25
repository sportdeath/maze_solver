#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped

from MapUtils import MapUtils
from RobotState import RobotState
from VisualizeLine import VisualizeLine
from RRT import RRT

class Test_RRT(VisualizeLine):
    def __init__(self):
        VisualizeLine.__init__(self,"Test_RRT")

        self.mapMsg = MapUtils.getMap()
        self.RRT = RRT(self.mapMsg)

        self.initialState = RobotState(0,0,3.14)
        
        self.clickSub = rospy.Subscriber(
                "/move_base_simple/goal", 
                PoseStamped, 
                self.clickedPose, 
                queue_size=1)

    def clickedPose(self, msg):
        # The received pose
        x = msg.pose.position.x
        y = msg.pose.position.y
        theta = MapUtils.quaternionToAngle(msg.pose.orientation)

        finalState = RobotState(x, y, theta)

        tree = self.RRT.computePath(self.initialState, finalState)

        # self.visualize(self.RRT.treeToLineList(tree),(1.,0.,0.),True)
        self.visualize(self.RRT.getPoints(),(0.,1.,0.))

if __name__=="__main__":
    rospy.init_node("Test_RRT")
    test_rrt = Test_RRT()
    rospy.spin()
