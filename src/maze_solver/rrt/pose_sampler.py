import numpy as np

import rospy

class PoseSampler:

    OCCUPANCY_THRESHOLD = rospy.get_param("/maze_solver/occupancy_threshold")
    P_UNIFORM = rospy.get_param("/maze_solver/p_uniform")
    P_GAUSSIAN = rospy.get_param("/maze_solver/p_gaussian")
    SEARCH_RADIUS = rospy.get_param("/maze_solver/search_radius")
    BRIDGE_STD_DEV = rospy.get_param("/maze_solver/bridge_std_dev")
    GAUSSIAN_STD_DEV = rospy.get_param("/maze_solver/gaussian_std_dev")

    @staticmethod
    def px_to_point(x_px, y_px, map_msg):
        x = x_px * map_msg.info.resolution + map_msg.info.origin.position.x
        y = y_px * map_msg.info.resolution + map_msg.info.origin.position.y
        return np.array([x, y])

    @staticmethod
    def point_to_px(point, map_msg):
        x = point[0] - map_msg.info.origin.position.x
        y = point[1] - map_msg.info.origin.position.y
        x_px = int(np.rint(x/map_msg.info.resolution))
        y_px = int(np.rint(y/map_msg.info.resolution))

        return x_px, y_px

    def occupied(self, point, map_msg):
        x, y = PoseSampler.point_to_px(point, map_msg)
        if (x < 0) or (y < 0) or (x >= map_msg.info.width) or (y >= map_msg.info.height):
            return False
        return map_msg.data[y, x] > self.OCCUPANCY_THRESHOLD

    def bridge(self, map_msg, occupied_points):
        while True:
            # Choose a random occupied point
            index = np.random.randint(len(occupied_points))
            point0_px = occupied_points[index]
            point0 = PoseSampler.px_to_point(point0_px[1], point0_px[0], map_msg) 
            # Pick a point nearby
            noise = np.random.normal(scale=self.BRIDGE_STD_DEV, size=2)
            point1 = point0 + noise
            
            # Check if it is occupied
            if self.occupied(point1, map_msg):

                # If it is find the midpoint
                midpoint = (point0 + point1)/2.

                # Check if it is unoccupied
                if not self.occupied(midpoint, map_msg):

                    # Choose an orientation that is 
                    # perpendicular to the bridge
                    diff = point1 - point0
                    theta = np.arctan2(diff[0], -diff[1])
                    pose = np.array([midpoint[0], midpoint[1], theta])
                    return pose

    def uniform(self, map_msg):
        while True:
            # Pick a point within the radius
            r_squared = np.random.uniform(0, self.SEARCH_RADIUS * self.SEARCH_RADIUS)
            r = np.sqrt(r_squared)
            phi = np.random.uniform(-np.pi,np.pi)
            x = r * np.cos(phi)
            y = r * np.sin(phi)

            # Check if it is occupied
            if not self.occupied((x,y), map_msg):

                # If not, add a random orientation
                theta = np.random.uniform(-np.pi, np.pi)
                pose = np.array([x, y, theta])
                return pose

    def gaussian(self, pose, map_msg):
        while True:
            # Pick a point around the pose
            noise = np.random.normal(scale=self.GAUSSIAN_STD_DEV, size=2)
            point = pose[:2] + noise

            # Check if it is occupied
            if not self.occupied(point, map_msg):

                # If not, add a random orientation
                theta = np.random.uniform(-np.pi, np.pi)
                pose = np.array([point[0], point[1], theta])
                return pose

    def hybrid(self, pose, map_msg, occupied_points):
        choice = np.random.uniform()
        if choice < self.P_UNIFORM or len(occupied_points) == 0:
            return self.uniform(map_msg)
        elif choice < self.P_UNIFORM + self.P_GAUSSIAN:
            return self.gaussian(pose, map_msg)
        else:
            return self.bridge(map_msg, occupied_points)
