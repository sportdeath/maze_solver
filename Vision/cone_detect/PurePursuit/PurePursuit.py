#!/usr/bin/env python

from CircleThreshold import CircleThreshold
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Image  # the rostopic message we subscribe
from cv_bridge import CvBridge # package to convert rosmsg<->cv2 
import numpy as np
import cv2


class PurePursuit:
    carLength = 32.5 # cm
    velocity = 2.5 # m/s
    reactionTime = 0.4  # sec
    lookAheadDistance = velocity * 100 * reactionTime

    def __init__(self):
        self.bridge = CvBridge()
        self.sub_image = rospy.Subscriber("/zed/rgb/image_rect_color",\
                Image, self.PureControl, queue_size=1)

        self.publisher = rospy.Publisher(\
                "/vesc/high_level/ackermann_cmd_mux/input/nav_0",\
                AckermannDriveStamped,\
                queue_size = 1)

        self.pub_image = rospy.Publisher("/echo_image",\
                Image, queue_size=1)

    def PureControl(self, image_msg):
        image_cv = self.bridge.imgmsg_to_cv2(image_msg)            

        ct = CircleThreshold(self.lookAheadDistance)
        goalPointWorld = ct.findGoalPoint(image_cv)

        steeringAngle = self.ackermannAngle(goalPointWorld)
        msg = AckermannDriveStamped()
        if steeringAngle:
            rospy.loginfo('steeringAngle = %f', steeringAngle)
            msg.drive.speed = self.velocity
            msg.drive.steering_angle = steeringAngle
        else:
            rospy.loginfo("no line!")

        self.publisher.publish(msg)

        #outputImage = cv2.bitwise_and( \
        #        image_cv, \
        #        image_cv, \
        #        mask = ct.mask)

        # convert cv2 message back to rosmsg
        #image_ros_msg = self.bridge.cv2_to_imgmsg(outputImage,"bgr8")

        # publish rosmsg 
        #self.pub_image.publish(image_ros_msg)

    def ackermannAngle(self, goalPointWorld):
        if goalPointWorld:
            goalDistance = np.linalg.norm(goalPointWorld)

            sinAlpha = goalPointWorld[0]/goalDistance
            curvature = 2*sinAlpha/goalDistance

            return -np.arctan(curvature * self.carLength)
        else:
            return None

if __name__=="__main__":
    rospy.init_node("PurePursuitNode") 
    PurePursuit()
    rospy.spin()
