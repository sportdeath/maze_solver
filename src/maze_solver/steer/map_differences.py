#!/usr/bin/env python2

"""
A node that subscribes to a map that is updated,
and publishes a list of all of the points that have
become occupied since the last update.
"""

import numpy as np

import rospy
from visualization_msgs.msg import Marker
from nav_msgs.msg import OccupancyGrid

from maze_solver.utils.visualization_utils import VisualizationUtils

class MapDifferences:

    OCCUPANCY_THRESHOLD = 0.2
    VISUALIZATION_TOPIC = "/line"
    CARTOGRAPHER_TOPIC = "/cartographer_map"
    CARTOGRAPHER_FRAME = "cartographer_map"

    def __init__(self):
        self.grid = None

        # Publish a list of points
        self.line_pub = rospy.Publisher(
                self.VISUALIZATION_TOPIC,
                Marker,
                queue_size=1)

        # Subscribe to the map
        self.map_sub = rospy.Subscriber(
                self.CARTOGRAPHER_TOPIC,
                OccupancyGrid,
                self.map_cb, 
                queue_size=1)

    def map_cb(self, msg):
        grid = np.array(msg.data)/100.
        grid = np.reshape(grid, (msg.info.height, msg.info.width))

        if self.grid is not None:
            # Find the index of the old map origin
            # in the new map
            origin_new = np.array((msg.info.origin.position.x, msg.info.origin.position.y))
            origin_old = np.array((self.info.origin.position.x, self.info.origin.position.y))
            origin_offset = origin_old - origin_new
            origin_indices = np.round(origin_offset / msg.info.resolution).astype(int)

            
            # Pad the old map
            x_before = origin_indices[0]
            x_after = msg.info.width - self.info.width - x_before
            y_before = origin_indices[1]
            y_after = msg.info.height - self.info.height - y_before
            paddings = ((np.maximum(0, y_before),
                         np.maximum(0, y_after)),
                        (np.maximum(0, x_before),
                         np.maximum(0, x_after)))
            self.grid = np.pad(self.grid, paddings, 'constant', constant_values=-1)

            # Clip the old map
            x_clip_before = np.maximum(0, -x_before)
            x_clip_after = msg.info.width + x_clip_before
            y_clip_before = np.maximum(0, -y_before)
            y_clip_after = msg.info.height + y_clip_before
            self.grid = self.grid[y_clip_before:y_clip_after, x_clip_before:x_clip_after]

            # Find points that have changed to occupied
            y, x = np.where(np.logical_and(grid >= self.OCCUPANCY_THRESHOLD, self.grid < self.OCCUPANCY_THRESHOLD))
            x = x * msg.info.resolution
            y = y * msg.info.resolution
            x += msg.info.origin.position.x
            y += msg.info.origin.position.y

            # Plot it
            VisualizationUtils.plot(
                    x, y,
                    self.line_pub, 
                    frame=self.CARTOGRAPHER_FRAME,
                    marker_type=Marker.POINTS)

        self.grid = grid
        self.info = msg.info

if __name__ == "__main__":
    rospy.init_node("map_differences")
    MapDifferences()
    rospy.spin()
