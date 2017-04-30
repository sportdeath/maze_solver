#!/usr/bin/env python

import rospy

from RobotState import RobotState
from VisualizeLine import VisualizeLine
from geometry_msgs.msg import PoseStamped
from Steer import Steer
import tf

class Test_Steer(VisualizeLine):
    def __init__(self):
        VisualizeLine.__init__(self,"Test_Steer",(1,0,0))
        self.clickSub = rospy.Subscriber(
                "/move_base_simple/goal", 
                PoseStamped, 
                self.clickedPose, 
                queue_size=1)

        self.initialState = RobotState (2,3,1)

    def clickedPose(self, msg):
        # The received pose
        x = msg.pose.position.x
        y = msg.pose.position.y
        theta = Test_Steer.quaternionToAngle(msg.pose.orientation)

        finalState = RobotState(x, y, theta)

        steer = Steer(self.initialState, finalState)

        if steer.isSteerable():
            self.visualize(steer.getPoints())

    @staticmethod
    def quaternionToAngle(q):
        """Convert a quaternion _message_ into an angle in radians.
        The angle represents the yaw.
        This is not just the z component of the quaternion."""
        x, y, z, w = q.x, q.y, q.z, q.w
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((x, y, z, w))
        return yaw


if __name__=="__main__":
    rospy.init_node("RRT Ros Node")
    test_steer = Test_Steer()
    rospy.spin()
