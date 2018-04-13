import numpy as np

class PoseSampler:

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

    @staticmethod
    def occupied(point, map_msg, occupancy_threshold):
        x, y = PoseSampler.point_to_px(point, map_msg)
        if (x < 0) or (y < 0) or (x >= map_msg.info.width) or (y >= map_msg.info.height):
            return False
        return map_msg.data[y, x] > occupancy_threshold

    @staticmethod
    def bridge(std_dev, map_msg, occupied_points, occupancy_threshold):
        while True:
            # Choose a random occupied point
            index = np.random.randint(len(occupied_points))
            point0_px = occupied_points[index]
            point0 = PoseSampler.px_to_point(point0_px[1], point0_px[0], map_msg)

            # Pick a point nearby
            noise = np.random.normal(scale=std_dev, size=2)
            point1 = point0 + noise
            
            # Check if it is occupied
            if PoseSampler.occupied(point1, map_msg, occupancy_threshold):

                # If it is find the midpoint
                midpoint = (point0 + point1)/2.

                # Check if it is unoccupied
                if not PoseSampler.occupied(midpoint, map_msg, occupancy_threshold):

                    # Add a random orientation
                    theta = np.random.uniform(-np.pi, np.pi)
                    pose = np.array([midpoint[0], midpoint[1], theta])
                    return pose

    @staticmethod
    def uniform(max_radius, map_msg, occupancy_threshold):
        while True:
            # Pick a point within the radius
            r_squared = np.random.uniform(0, max_radius * max_radius)
            r = np.sqrt(r_squared)
            phi = np.random.uniform(-np.pi,np.pi)
            x = r * np.cos(phi)
            y = r * np.sin(phi)

            # Check if it is occupied
            if not PoseSampler.occupied((x,y), map_msg, occupancy_threshold):

                # If not, add a random orientation
                theta = np.random.uniform(-np.pi, np.pi)
                pose = np.array([x, y, theta])
                return pose

    @staticmethod
    def hybrid(p_uniform, max_radius, bridge_std_dev, map_msg, occupied_points, occupancy_threshold):
        choice = np.random.uniform()
        if choice < p_uniform:
            return PoseSampler.uniform(max_radius, map_msg, occupancy_threshold)
        else:
            return PoseSampler.bridge(bridge_std_dev, map_msg, occupied_points, occupancy_threshold)
