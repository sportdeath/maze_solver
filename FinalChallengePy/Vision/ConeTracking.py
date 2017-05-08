#!/usr/bin/env python

import cv2  # the open cv library packge
import rospy # standard ros with python package
from sensor_msgs.msg import Image  # the rostopic message we subscribe/publish 
from final_challenge.msg import ConeInfo
from cv_bridge import CvBridge # package to convert rosmsg<->cv2 
from CircleThreshold import CircleThreshold
from ConeThreshold import ConeThreshold
dsf
import numpy as np

class ConeTracking:
    IMAGE_HEIGHT = 376.  # pixel height
    IMAGE_WIDTH = 672.  # pixel width

    HSV_MIN_ORANGE = np.array([2, 182, 133])
    HSV_MAX_ORANGE = np.array([7, 254, 225])

    HSV_MIN_GREEN = np.array([74, 73, 39])
    HSV_MAX_GREEN = np.array([109, 185, 85])

    RED_CONE_DIRECTION = 1
    GREEN_CONE_DIRECTION = -RED_CONE_DIRECTION

    def __init__(self):
        
        # create bridge to convert to and from cv2 and rosmsg
        self.bridge = CvBridge()

        # Uncomment for debugging purposes
        # self.pubImage = rospy.Publisher("/masked_image",\
        #         Image, queue_size=1)

        self.pubConeInfo = rospy.Publisher("/cone_info", ConeInfo, queue_size=1)

        # subscribe to the rostopic carrying the image we are interested in
        # "camera/rgb/image_rect_color" is the topic name
        # Image is the message type
        # self.processImage is the callback function executed every time we
        # recieve the message
        self.subImage = rospy.Subscriber("/zed/rgb/image_rect_color",\
                Image, self.processImage, queue_size=1)

        self.coneLocList = []

        # report initalization success
        rospy.loginfo("Cone Tracking Initialized.")

    """
    This is the callback function that is executed every time an Image
    message is received on the rostopic we subscribed to.
    """
    def processImage(self, image_msg):
        rospy.loginfo("Processing Image")

        # convert rosmsg to cv2 type
        imageCv = self.bridge.imgmsg_to_cv2(image_msg)

        redThreshold = ConeThreshold(imageCv, self.HSV_MIN_ORANGE, self.HSV_MAX_ORANGE)
        greenThreshold = ConeThreshold(imageCv, self.HSV_MIN_GREEN, self.HSV_MAX_GREEN)
        
        # Uncomment for debugging purposes
        # outputCombined = cv2.add(redThreshold.getBoundedImage("red"), greenThreshold.getBoundedImage("green"))

        # convert cv2 message back to rosmsg
        # image_ros_msg = self.bridge.cv2_to_imgmsg(outputCombined,"bgr8")

        # publish rosmsg 
        # self.pubImage.publish(image_ros_msg)

        redCalculations = ConeCalculations(redThreshold.getBottomCenterPoint())
        greenCalculations = ConeCalculations(greenThreshold.getBottomCenterPoint())

        # There is probably a better way to do this
        if redCalculations.isNewCone(self.coneLocList):
            msg = ConeInfo()
            msg.direction = self.RED_CONE_DIRECTION

            worldCoordinates = redCalculations.getWorldCoordinatesAsPoint()
            msg.location = worldCoordinates

            coneLocList.append(np.array(worldCoordinates.x, worldCoordinates.y))

            self.pubConeInfo.publish(msg)

        if greenCalculations.isNewCone(self.coneLocList):
            msg = ConeInfo()
            msg.direction = self.GREEN_CONE_DIRECTION
            
            worldCoordinates = greenCalculations.getWorldCoordinatesAsPoint()
            msg.location = worldCoordinates

            coneLocList.append(np.array(worldCoordinates.x, worldCoordinates.y))

            self.pubConeInfo.publish(msg)

if __name__=="__main__":
    rospy.init_node('ConeTracking')

    coneTracking = ConeTracking()

    rospy.spin()

