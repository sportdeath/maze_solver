#!/usr/bin/env python2

import abc
import dubins

import numpy as np

OCCUPANCY_THRESHOLD = 80

class Steer(object):
    __metaclass__ = abc.ABCMeta

    @abc.abstractmethod
    def sample(self, step_size):
        pass

    @abc.abstractmethod
    def length(self):
        pass

    def intersects(self, sample_width, map_msg):
        # Compute the necessary sample width
        # to account for all intersections

        return self.sample_intersects(sample_width, map_msg)


        # sample_width = 0.8

        # Extract the points
        # points_px = self.sample(sample_width, map_msg)
        poses = self.sample(sample_width, map_msg)
        # return False
        # return points_px

        # poses = np.zeros((30,3))

        # Convert to map coordinates
        points = poses[:, :2]
        points[:,0] -= map_msg.info.origin.position.x
        points[:,1] -= map_msg.info.origin.position.y
        points /= map_msg.info.resolution
        np.rint(points, out=points)
        points_px = points.astype(int)
        # points_px = int(points)
        return False

        # Remove ones that are out of bounds
        upper_points = (points_px < np.array([map_msg.info.width, map_msg.info.height]))
        lower_points = (points_px >= np.zeros((2)))
        good_points = np.logical_and(np.all(upper_points, axis=1), np.all(lower_points, axis=1))
        points_px = points_px[good_points]

        # Fetch the values at those coordinates
        occupancy_values = map_msg.data[points_px[:,1],points_px[:,0]]

        does_intersect = np.any(occupancy_values > OCCUPANCY_THRESHOLD)

        return does_intersect

class DubinsSteer(Steer):

    def __init__(self, q0, q1, turning_radius):
        self.q1 = q1
        self.path = dubins.shortest_path(q0, q1, turning_radius)

    def sample(self, step_size):
        # poses = self.path.sample_many_map(
                # step_size, 
                # map_msg.info.origin.position.x,
                # map_msg.info.origin.position.y,
                # map_msg.info.resolution)

        poses,_ = self.path.sample_many(step_size)
        poses.append(self.q1)
        poses = np.array(poses)
        return poses

    def sample_intersects(self, step_size, map_msg):
        return self.path.sample_intersects(step_size, map_msg, OCCUPANCY_THRESHOLD)

    def length(self):
        return self.path.path_length()

class CircleSteer(Steer):

    def __init__(self, length, radius):
        self.length = length
        self.radius = radius
        
    def sample(step_size):
        pass

    def length(self):
        return self.length

if __name__ == "__main__":
    from rospy.numpy_msg import numpy_msg
    from nav_msgs.msg import OccupancyGrid
    from maze_solver.rrt.rrt_node import RRTNode
    from timeit import default_timer as timer

    m = numpy_msg(OccupancyGrid)()
    m.info.resolution = 0.05
    m.data = np.zeros((1000, 1000), dtype=np.int8)
    m.info.origin.position.x = 2.
    m.info.origin.position.y = 1.5
    m.info.height = m.data.shape[0]
    m.info.width = m.data.shape[1]
    radius = m.data.shape[0] * 0.05 + 3.
    turning_radius = 1.

    s = 0
    n = 0
    while True:
        q0 = RRTNode(radius).pose
        q1 = RRTNode(radius).pose
        steer = DubinsSteer(q0, q1, turning_radius)
        start = timer()
        steer.intersects(m, 0.3, 0.5)
        end = timer()
        s += (end - start)
        n += 1
        if n % 1000 == 0:
            print "Average running time: ", s/n
