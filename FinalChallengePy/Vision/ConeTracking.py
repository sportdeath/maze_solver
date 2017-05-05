#!/usr/bin/env python

import cv2  # the open cv library packge
import rospy # standard ros with python package
from sensor_msgs.msg import Image  # the rostopic message we subscribe/publish 
from final_challenge.msg import ConeInfo
from cv_bridge import CvBridge # package to convert rosmsg<->cv2 
from CircleThreshold import CircleThreshold
import numpy as np

class ConeTracking:
    IMAGE_HEIGHT = 376.  # pixel height
    IMAGE_WIDTH = 672.  # pixel width

    def __init__(self):
        
        # create bridge to convert to and from cv2 and rosmsg
        self.bridge = CvBridge()

        # Comment back in for debugging

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
        circleThreshold = CircleThreshold(imageCv)

        # Uncomment for debugging purposes
        # outputImage = threshold.getBoundedImage()
        # outputImage = threshold.getMatchedImage()

        # convert cv2 message back to rosmsg
        # image_ros_msg = self.bridge.cv2_to_imgmsg(outputImage,"bgr8")

        # publish rosmsg 
        # self.pubImage.publish(image_ros_msg)

        if circleThreshold.newCone():
            msg = ConeInfo()
            msg.direction = circleThreshold.getDirection()
            msg.location = circleThreshold.getWorldCoordinatesAsPoint()

            self.pubConeInfo.publish(msg)

if __name__=="__main__":
    # initalize the ros node
    rospy.init_node('ConeTracking')

    # create Echo to start the image passthrough
    coneTracking = ConeTracking()

    # continue running echo until node is killed from outside process
    rospy.spin()

