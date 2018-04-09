#!/usr/bin/env python2

"""
A node that subscribes to a map that is updated,
and publishes a list of all of the points that have
become occupied since the last update.
"""

import numpy as np

from rtree import index as rtree_index

import rospy
from visualization_msgs.msg import Marker
from nav_msgs.msg import OccupancyGrid, MapMetaData

from maze_solver.utils.visualization_utils import VisualizationUtils

class MapDifferences:

    CAR_RADIUS = 0.3
    OCCUPANCY_THRESHOLD = 0.5
    OCCUPIED_TOPIC = "/occupied"
    UNOCCUPIED_TOPIC = "/unoccupied"
    CARTOGRAPHER_TOPIC = "/cartographer_map"
    CARTOGRAPHER_FRAME = "cartographer_map"

    def __init__(self):
        self.grid = -np.ones((1, 1))
        self.info = MapMetaData()
        self.info.width = 1
        self.info.height = 1

        self.rtree = rtree_index.Index()

        # Publish a list of points
        self.occupied_pub = rospy.Publisher(
                self.OCCUPIED_TOPIC,
                Marker,
                queue_size=1)
        self.unoccupied_pub = rospy.Publisher(
                self.UNOCCUPIED_TOPIC,
                Marker,
                queue_size=1)

        # Publish to the map
        self.map_pub = rospy.Publisher(
                "/cartographer_map_conv",
                OccupancyGrid,
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
        y, x = np.where(np.logical_and(
            grid >= self.OCCUPANCY_THRESHOLD, 
            self.grid < self.OCCUPANCY_THRESHOLD))
        x += np.round(msg.info.origin.position.x/msg.info.resolution).astype(int)
        y += np.round(msg.info.origin.position.y/msg.info.resolution).astype(int)
        x_map = x * msg.info.resolution
        y_map = y * msg.info.resolution

        # Add them to the rtree
        for xi, yi, xi_map, yi_map in zip(x, y, x_map, y_map):
            i = xi + 10000 * yi
            insertion = (
                    xi_map - self.CAR_RADIUS,
                    yi_map - self.CAR_RADIUS,
                    xi_map + self.CAR_RADIUS,
                    yi_map + self.CAR_RADIUS)
            self.rtree.insert(i, insertion)

        # Plot it
        if self.occupied_pub.get_num_connections() > 0:
            VisualizationUtils.plot(
                    x_map, y_map,
                    self.occupied_pub, 
                    frame=self.CARTOGRAPHER_FRAME,
                    marker_type=Marker.POINTS)

        # Find points that are no longer occupied
        y, x = np.where(np.logical_and(
            grid < self.OCCUPANCY_THRESHOLD, 
            self.grid >= self.OCCUPANCY_THRESHOLD))
        x += np.round(msg.info.origin.position.x/msg.info.resolution).astype(int)
        y += np.round(msg.info.origin.position.y/msg.info.resolution).astype(int)
        x_map = x * msg.info.resolution
        y_map = y * msg.info.resolution

        # Add them to the rtree
        for xi, yi, xi_map, yi_map in zip(x, y, x_map, y_map):
            i = xi + 10000 * yi
            insertion = (
                    xi_map - self.CAR_RADIUS,
                    yi_map - self.CAR_RADIUS,
                    xi_map + self.CAR_RADIUS,
                    yi_map + self.CAR_RADIUS)
            self.rtree.delete(i, insertion)

        # Plot it
        if self.unoccupied_pub.get_num_connections() > 0:
            VisualizationUtils.plot(
                    x_map, y_map,
                    self.unoccupied_pub, 
                    color=(0., 1., 0.),
                    frame=self.CARTOGRAPHER_FRAME,
                    marker_type=Marker.POINTS)

        self.grid = grid
        self.info = msg.info

if __name__ == "__main__":
    rospy.init_node("map_differences")
    MapDifferences()
    rospy.spin()
