import rospy
import numpy as np

from std_msgs.msg import Header
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
import tf.transformations
import tf

class VisualizeLine:
    def __init__(self, topicName, color):
        self.publisher = rospy.Publisher(
                topicName,
                Marker,
                queue_size=1)

        self.transformationFramePublisher =\
                tf.TransformBroadcaster()

        self.color = color

        # r = rospy.Rate(10)

        # while not rospy.is_shutdown():
        #     self.visualize(points, color)
        #     self.publishTransformationFrame()
        #     r.sleep()

    def visualize(self, points):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "/base_link"

        lineStrip = Marker()
        lineStrip.type = Marker.LINE_STRIP
        lineStrip.action = Marker.ADD
        lineStrip.header = header
        lineStrip.scale.x = 0.1
        lineStrip.pose.orientation.w = 1.
        lineStrip.color.a = 1.
        lineStrip.color.r = self.color[0]
        lineStrip.color.g = self.color[1]
        lineStrip.color.b = self.color[2]

        for point in points:
            p = Point()
            p.x = point[0]
            p.y = point[1]
            p.z = 1
            lineStrip.points.append(p)

        self.publisher.publish(lineStrip)

        self.publishTransformationFrame()

    def publishTransformationFrame(self):
        """ Publish a tf from map to base_link. """
        stamp = rospy.Time.now()

        position = (0, 0, 0)
        orientation = (0,0,0,1)

        self.transformationFramePublisher.sendTransform(
                position, 
                orientation, 
                stamp, 
                "/base_link", 
                "/map");
