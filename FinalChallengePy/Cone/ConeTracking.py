#!/usr/bin/env python

import cv2  # the open cv library packge
import rospy # standard ros with python package
from sensor_msgs.msg import Image  # the rostopic message we subscribe/publish 
from final_challenge.msg import ConeInfo
from cv_bridge import CvBridge # package to convert rosmsg<->cv2 

from FinalChallengePy.Cone.ConeThreshold import ConeThreshold
from FinalChallengePy.Cone.Cone import Cone
from FinalChallengePy.Cone.Constants import *

from FinalChallengePy.PathPlanning.RobotState import RobotState

import numpy as np

class ConeTracking:
    
    referenceState = RobotState(0.,0.,0.)

    # Set coordinates for setpoint control line follower
    IMAGE_HEIGHT = 376.  # pixel height
    IMAGE_WIDTH = 672.  # pixel width
    M_X = 0. # start at left side of image
    M_Y = 0.15*IMAGE_HEIGHT # start from 15% from top of image
    M_HEIGHT = 0.85*IMAGE_HEIGHT # go until bottom of image
    M_WIDTH = IMAGE_WIDTH # sample entire width

    def __init__(self):
        self.redCones = []
        self.greenCones = []
        
        # create bridge to convert to and from cv2 and rosmsg
        self.bridge = CvBridge()

        # Uncomment for debugging purposes
        self.pubImage = rospy.Publisher("/masked_image",\
                Image, queue_size=1)

        self.pubConeInfo = rospy.Publisher(
                "/cone_info", 
                ConeInfo, 
                queue_size=1)

        # subscribe to the rostopic carrying the image we are interested in
        # "camera/rgb/image_rect_color" is the topic name
        # Image is the message type
        # self.processImage is the callback function executed every time we
        # recieve the message
        self.subImage = rospy.Subscriber("/zed/rgb/image_rect_color",\
                Image, self.processImage, queue_size=1)

        # report initalization success
        rospy.loginfo("Cone Tracking Initialized.")

    """
    This is the callback function that is executed every time an Image
    message is received on the rostopic we subscribed to.
    """
    def processImage(self, image_msg):
        # convert rosmsg to cv2 type
        imageCv = self.bridge.imgmsg_to_cv2(image_msg)

        # mask top and bottom of image
        manualMask = np.zeros(imageCv.shape, np.uint8)
        manualMask[M_Y:M_Y+M_HEIGHT, M_X:M_X+M_WIDTH] = imageCv[M_Y:M_Y+M_HEIGHT, M_X:M_X+M_WIDTH]

        # add color thresholding
        redThreshold = ConeThreshold(manualMask, HSV_MIN_ORANGE, HSV_MAX_ORANGE)
        greenThreshold = ConeThreshold(manualMask, HSV_MIN_GREEN, HSV_MAX_GREEN)
        
        # Uncomment for debugging purposes
        outputCombined = cv2.add(
            redThreshold.getBoundedImage("red"), 
            greenThreshold.getBoundedImage("green"))

        # convert cv2 message back to rosmsg
        image_ros_msg = self.bridge.cv2_to_imgmsg(outputCombined,"bgr8")

        # publish rosmsg 
        self.pubImage.publish(image_ros_msg)

        if redThreshold.doesExist():
            redCone = Cone(redThreshold.getBottomCenterPoint())
            if np.linalg.norm(redCone.getPosition()) < CONE_VISIBILITY_THRESHOLD:
                self.updateCones(redCone, self.redCones, RED_CONE_DIRECTION)

        if greenThreshold.doesExist():
            greenCone = Cone(greenThreshold.getBottomCenterPoint())
            if np.linalg.norm(greenCone.getPosition()) < CONE_VISIBILITY_THRESHOLD:
                self.updateCones(greenCone, self.greenCones, GREEN_CONE_DIRECTION)
    
    def updateCones(self, cone, cones, direction):
        #if cone.isNewCone(cones):
        msg = ConeInfo()
        msg.direction = direction
        worldCoordinates = cone.getPosition()
        msg.location.x = worldCoordinates[0]
        msg.location.y = worldCoordinates[1]
        self.pubConeInfo.publish(msg)
        # cones.append(cone)

if __name__=="__main__":
    rospy.init_node('ConeTracking')
    coneTracking = ConeTracking()
    rospy.spin()

