#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, PoseArray

from FinalChallengePy.Utils.VisualizeLine import VisualizeLine
from FinalChallengePy.Utils.RobotState import RobotState
from FinalChallengePy.Utils.MapUtils import MapUtils
from FinalChallengePy.Utils.GeomUtils import GeomUtils

class Test_AngleConversions(VisualizeLine):

    def __init__(self):
        VisualizeLine.__init__(self,"Test", numPublishers = 1)

        self.samplePub  = rospy.Publisher(
                "/samples", 
                PoseArray, 
                queue_size = 1)

        self.goalSub = rospy.Subscriber(
                "/move_base_simple/goal", 
                PoseStamped, 
                self.receivedPose, 
                queue_size=1)

    def receivedPose(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        theta = MapUtils.quaternionToAngle(msg.pose.orientation)

        rospy.loginfo("Theta: " + str(theta))

        # Make it into a robot state and plot the vector
        state = RobotState(x, y, theta)

        MapUtils.publishStates([state], self.samplePub)

        points = [state.getPosition(), state.getPosition() + state.getOrientation()]

        self.visualize(points, (0., 0., 1))

        # Turn it back into an angle and publish a pose

        thetaAfter = GeomUtils.getAngle(state.getOrientation())

        rospy.loginfo("Theta difference = " + str(theta - thetaAfter) + str(" (this should be zero)"))

if __name__=="__main__":
    rospy.init_node("Test_AngleConversion")
    test_angleConversions = Test_AngleConversions()
    rospy.spin()
