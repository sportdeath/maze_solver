import abc
import dubins

import numpy as np

OCCUPANCY_THRESHOLD = 50

class Steer(object):
    __metaclass__ = abc.ABCMeta

    @abc.abstractmethod
    def sample(self, step_size):
        pass

    @abc.abstractmethod
    def length(self):
        pass

    def intersects(self, map_msg):
        # Extract the points
        poses = self.sample(map_msg.info.resolution)
        points = poses[:, :2]

        # Convert to map coordinates
        points -= np.array((map_msg.info.origin.position.x, map_msg.info.origin.position.y))
        points_px = np.round(points/map_msg.info.resolution).astype(int)

        # Fetch the values at those coordinates
        occupancy_values = map_msg.data[points_px[:,1],points_px[:,0]]

        return np.any(occupancy_values > OCCUPANCY_THRESHOLD)

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
