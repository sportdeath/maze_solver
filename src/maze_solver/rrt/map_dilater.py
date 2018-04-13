#!/usr/bin/env python2

"""
This node subscribes to a map published by
cartographer and dilates it.
"""

import numpy as np
import skimage.morphology

import rospy
from rospy.numpy_msg import numpy_msg
from nav_msgs.msg import OccupancyGrid

class MapDilater:

    DILATION_RADIUS = rospy.get_param("/maze_solver/dilation_radius")
    CARTOGRAPHER_TOPIC = rospy.get_param("/maze_solver/cartographer_topic")
    CARTOGRAPHER_DILATED_TOPIC = rospy.get_param("/maze_solver/cartographer_dilated_topic")

    def __init__(self):
        # Initialize space for the dilated grid
        self.grid_dilated = np.zeros((1, 1), dtype=np.int8)

        # Publish to the map
        self.map_pub = rospy.Publisher(
                self.CARTOGRAPHER_DILATED_TOPIC,
                numpy_msg(OccupancyGrid),
                latch=True,
                queue_size=1)

        # Subscribe to the map
        self.map_sub = rospy.Subscriber(
                self.CARTOGRAPHER_TOPIC,
                numpy_msg(OccupancyGrid),
                self.map_cb, 
                queue_size=1)

    def dilate(self, map_msg):
        # Apply dilation
        # self.grid_dilated.resize(map_msg.data.shape)
        disk = skimage.morphology.disk(np.ceil(self.DILATION_RADIUS/map_msg.info.resolution), dtype=np.int8)
        # skimage.morphology.dilation(map_msg.data, selem=disk, out=self.grid_dilated)
        map_msg.data = skimage.morphology.dilation(map_msg.data, selem=disk)

        # Publish the message
        # map_msg.data = self.grid_dilated
        map_msg.data.shape = (-1)
        self.map_pub.publish(map_msg)
        # return map_msg

    def map_cb(self, map_msg):
        map_msg.data.shape = (map_msg.info.height, map_msg.info.width)

        map_msg = self.dilate(map_msg)

if __name__ == "__main__":
    rospy.init_node("map_dilater")
    MapDilater()
    rospy.spin()
