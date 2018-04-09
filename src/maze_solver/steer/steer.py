import abc
import dubins

import numpy as np

class Steer(object):
    __metaclass__ = abc.ABCMeta

    @abc.abstractmethod
    def sample(self, step_size):
        pass

    @abc.abstractmethod
    def length(self):
        pass

    def intersects(self, robot_radius, map_msg):
        # Extract the points
        poses = self.sample(robot_radius)
        points = poses[:, :2]

        # Convert to map coordinates
        points -= np.array((map_msg.info.height, map_msg.info.width))
        points_px = np.round(points/map_msg.info.resolution).astype(int)
        robot_radius_px = np.ceil(robot_radius/map_msg.info.resolution).astype(int)

        # Add the kernel
        points_px = np.reshape(points_px, (points.shape[0], 1, 1, 2))
        kernel_offsets = np.arange(-robot_radius_px, robot_radius_px, dtype=int)
        x, y = np.meshgrid(kernel_offsets, kernel_offsets)
        kernel = np.stack((x, y), axis=2)
        kernel = np.expand_dims(kernel, axis=0)

        points_px_kernelized = points_px + kernel
        points_px_kernelized = np.reshape(points_kernelized, (-1, 2))

        # Check each point

        # interects_ = False
        # for point in points:
            # num_hits = self.map_differences.rtree.count((point[0], point[1], point[0], point[1]))
            # intersects_ = intersects_ or (num_hits > 0)

        """
        Time intersection time with rtree
        or just doing it manually

        convert points to map pixels
        for each point also consider n points around that point
        """

        return intersects_

class DubinsSteer(Steer):

    def __init__(self, q0, q1, turning_radius):
        self.q1 = q1
        self.path = dubins.shortest_path(q0, q1, turning_radius)

    def sample(self, step_size):
        points, _ = self.path.sample_many(step_size)
        points.append(self.q1)
        points = np.array(points)
        return points

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
    ds = DubinsSteer((0, 0, 0), (1, 1, 1), 1.)
    ds.intersects(0.3, None)
