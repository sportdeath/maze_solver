#!/usr/bin/env python2

"""
A node that subscribes to a map that is updated,
and publishes a list of all of the points that have
become occupied since the last update.
"""

import numpy as np
import skimage.morphology

from rtree import index as rtree_index

import rospy
from rospy.numpy_msg import numpy_msg
from visualization_msgs.msg import Marker
from nav_msgs.msg import OccupancyGrid, MapMetaData

from maze_solver.utils.visualization_utils import VisualizationUtils

class MapDifferences:

    DILATION_RADIUS = 0.4
    OCCUPANCY_THRESHOLD = 0.5
    OCCUPIED_TOPIC = "/occupied"
    CARTOGRAPHER_TOPIC = "/cartographer_map"
    CARTOGRAPHER_DILATED_TOPIC = "/cartographer_map_dilated"
    CARTOGRAPHER_FRAME = "cartographer_map"

    def __init__(self):
        # Initialize an empty message
        self.data = -np.ones((1, 1))
        self.info = MapMetaData()
        self.info.width = 1
        self.info.height = 1

        # Initialize space for the dilated grid
        self.grid_dilated = np.zeros((1, 1), dtype=np.int8)

        # Publish a list of occupied points
        self.occupied_pub = rospy.Publisher(
                self.OCCUPIED_TOPIC,
                Marker,
                queue_size=1)

        # Publish to the map
        self.map_pub = rospy.Publisher(
                self.CARTOGRAPHER_DILATED_TOPIC,
                numpy_msg(OccupancyGrid),
                queue_size=1)

        # Subscribe to the map
        self.map_sub = rospy.Subscriber(
                self.CARTOGRAPHER_TOPIC,
                numpy_msg(OccupancyGrid),
                self.map_cb, 
                queue_size=1)

    def dilate(self, msg):
        # Apply dilation
        self.grid_dilated.resize(msg.data.shape)
        disk = skimage.morphology.disk(np.ceil(self.DILATION_RADIUS/msg.info.resolution), dtype=np.int8)
        skimage.morphology.dilation(msg.data, selem=disk, out=self.grid_dilated)

        # Publish the message
        msg_out = numpy_msg(OccupancyGrid)()
        msg_out.info = msg.info
        msg_out.header = msg.header
        msg_out.data = self.grid_dilated
        msg_out.data.shape = (-1)
        self.map_pub.publish(msg_out)

    def find_occupied(self, msg):
        # Find the index of the old map origin in the new map
        origin_new = np.array((msg.info.origin.position.x, msg.info.origin.position.y))
        origin_old = np.array((self.info.origin.position.x, self.info.origin.position.y))
        origin_offset = origin_old - origin_new
        origin_indices = np.round(origin_offset / msg.info.resolution).astype(int)

        if np.any(origin_indices != 0) or \
                msg.info.height != self.info.height or \
                msg.info.width != self.info.width:
            # Pad the old map
            x_before = origin_indices[0]
            x_after = msg.info.width - self.info.width - x_before
            y_before = origin_indices[1]
            y_after = msg.info.height - self.info.height - y_before
            paddings = ((np.maximum(0, y_before),
                         np.maximum(0, y_after)),
                        (np.maximum(0, x_before),
                         np.maximum(0, x_after)))
            self.data = np.pad(self.data, paddings, 'constant', constant_values=-1)

            # Clip the old map
            x_clip_before = np.maximum(0, -x_before)
            x_clip_after = msg.info.width + x_clip_before
            y_clip_before = np.maximum(0, -y_before)
            y_clip_after = msg.info.height + y_clip_before
            self.data = self.data[y_clip_before:y_clip_after, x_clip_before:x_clip_after]

        # Find points that have changed to occupied
        y, x = np.where(np.logical_and(
            msg.data >= self.OCCUPANCY_THRESHOLD, 
            self.data < self.OCCUPANCY_THRESHOLD))
        x = x * msg.info.resolution
        y = y * msg.info.resolution
        x += msg.info.origin.position.x
        y += msg.info.origin.position.y

        # Plot it
        if self.occupied_pub.get_num_connections() > 0:
            VisualizationUtils.plot(
                    x, y,
                    self.occupied_pub, 
                    frame=self.CARTOGRAPHER_FRAME,
                    marker_type=Marker.POINTS)

        self.data = msg.data
        self.info = msg.info

    def map_cb(self, msg):
        msg.data.shape = (msg.info.height, msg.info.width)

        self.dilate(msg)
        self.find_occupied(msg)

if __name__ == "__main__":
    rospy.init_node("map_differences")
    MapDifferences()
    rospy.spin()
