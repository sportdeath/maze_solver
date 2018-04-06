#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped

from FinalChallengePy.PathPlanning.RRT import RRT

from FinalChallengePy.Utils.MapUtils import MapUtils
from FinalChallengePy.Utils.VisualizeLine import VisualizeLine
from FinalChallengePy.Utils.RobotState import RobotState

class Test_RRT(VisualizeLine):
    def __init__(self):
        VisualizeLine.__init__(self,"Test",numPublishers=3)

        self.mapMsg = MapUtils.getMap()
        self.RRT = RRT(self.mapMsg, verbose = True)

        self.goalState = RobotState(-1,0,3.14)
        
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

        initState = RobotState(x, y, theta)

        bestGoalIndex, tree = self.RRT.computePath(initState, self.goalState, backwards=True)

        self.visualize(self.RRT.treeToLineList(tree),(0.,0.,0.7),publisherIndex=2,lineList=True)
        (forwardPoints, backwardsPoints) = self.RRT.getLineLists()
        self.visualize(forwardPoints,(0.,1.,0.),publisherIndex=0,lineList=True)
        self.visualize(backwardsPoints,(1.,0.,0.),publisherIndex=1,lineList=True)

if __name__=="__main__":
    rospy.init_node("Test_RRT")
    test_rrt = Test_RRT()
    rospy.spin()
