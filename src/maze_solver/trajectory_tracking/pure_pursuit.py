import numpy as np

class PurePursuit:
    """
    Methods for pure pursuit.
    """

    @staticmethod
    def ackermann_angle(goal_point_local, axle_length):
	"""
	Computes the steering angle of the car
	for a goal point relative to the racecar's
	frame of reference.

        Args:
            goal_point_local: The goal point relative the
                the racecar.
        
        Returns:
            steering_angle: The angle the car should steer
                at in order be on a circle to the goal point.
	"""

        goal_dist = np.linalg.norm(goal_point_local)

        sin_alpha = -goal_point_local[1]/goal_dist
        curvature = 2 * sin_alpha/goal_dist

        return -np.arctan(curvature * axle_length)


    @staticmethod
    def pick_closest_point(pose, points, prev_index, look_ahead_distance_sq):
        """
        Returns a point, interpolated from a list of points
        that is closest to the look ahead distance
        and the index of that point.

        Args:
            pose: The current pose of the racecar.
            points: A numpy array of 2D trajectory points.
            prev_index: The previous trajectory index of the racecar.
            look_ahead_distance_sq: The look ahead distance, squared.

        Returns:
            point: The trajectory point to steer to.
            index: The index of the point just before the
                interpolated point.
            is_lost: A boolean that is true if the racecar is
                off of the trajectory.
        """
        position = pose[:2]

        # Check to see if we are lost
        prev_diff = points[prev_index] - position
        prev_diff_sq = np.dot(prev_diff, prev_diff)
        if prev_diff_sq >= look_ahead_distance_sq:
            return points[prev_index], prev_index, True, False

        # For all future points
        for i in xrange(prev_index + 1, len(points)):
            # Search for the first point that is greater than
            # the look ahead distance
            diff = points[i] - position
            if np.dot(diff, diff) >= look_ahead_distance_sq:

                # Interpolate between the point just greater than (i)
                # and the point just less than (i - 1) the look ahead distance
                point_diff = points[i-1] - points[i]
                a = np.dot(point_diff, point_diff)
                b = 2 * np.dot(point_diff, diff)
                c = np.dot(point_diff, diff) - look_ahead_distance_sq

                root = (-b - np.sqrt(b*b - 4 * a * c))/(2 * a)

                interpolated = points[i] + root * point_diff

                return interpolated, i - 1, False, False

        # If all future points are less than the look ahead distance,
        # we have reached the end of the path.
        return points[-1], len(points) - 1, False, True
