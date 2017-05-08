#!/usr/bin/env python

''' SafetyTest.py
straight trajectory command
tests safety controller because no joystick needed
and the joystick overrides safety controller anyways
'''

import rospy

from ackermann_msgs.msg import AckermannDriveStamped

class SafetyTest:
    def __init__(self):

        self.commandPub = rospy.Publisher(
            "/vesc/high_level/ackermann_cmd_mux/input/nav_0",
            AckermannDriveStamped, queue_size = 1)

        rospy.sleep(1)

        while self.commandPub.get_num_connections() > 0:
            self.drive()
            rospy.sleep(0.2)

    def drive(self):
        message = AckermannDriveStamped()
        message.drive.speed = 2
        message.drive.steering_angle = 0
        
        self.commandPub.publish(message)

if __name__=="__main__":
    rospy.init_node("SafetyTest")
    safetyTest = SafetyTest()
    rospy.spin()