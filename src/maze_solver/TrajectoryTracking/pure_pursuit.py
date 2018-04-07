import numpy as np

class PurePursuit:
    """
    Methods for pure pursuit.
    """

    CAR_AXEL_DISTANCE = rospy.get_param("car_axle_distance")
    LOOK_AHEAD_DIST_SQUARED = rospy.get_param("look_ahead_dist_squared")

    def get_ackermann_angle(self, goal_point_local):
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

        sin_alpha = goal_point_local[0]/goal_distance
        curvature = 2 * sin_alpha/goal_distance

        return -np.arctan(curvature * self.CAR_AXLE_DISTANCE)


    def pick_closest_point(self, position, points, prev_index):
        """
        Returns a point, interpolated from a list of points
        that is closest to the look ahead distance
        and the index of that point.

        Args:
            position: The current position of the racecar.
            points: A numpy array of 2D trajectory points.
            prev_index: The previous trajectory index of the racecar.

        Returns:
            point: The trajectory point to steer to.
            index: The index of the point just before the
                interpolated point.
            is_lost: A boolean that is true if the racecar is
                off of the trajectory.
        """

        # If the current point index is further ahead than the
        # look ahead distance then return that point
        prev_diff = points[prev_index] - position
        prev_diff_squared = np.dot(prev_difference, prev_difference)
        if prev_difference_squared >= self.LOOK_AHEAD_DIST_SQUARED:
            return points[prev_index], prev_index, True


        # For all future points
        for i in xrange(prev_index + 1, len(points)):
            # Search for the first point that is greater than
            # the look ahead distance
            diff = points[i] - position
            if np.dot(diff, diff) >= self.LOOK_AHEAD_DIST_SQUARED:

                # Interpolate between the point just greater than (i)
                # and the point just less than (i - 1) the look ahead distance
                point_diff = points[i-1] - points[i]
                a = np.dot(point_diff, point_diff)
                b = 2 * np.dot(point_diff, diff)
                c = np.dot(point_diff, diff) - self.LOOK_AHEAD_DIST_SQUARED

                root = (-b - np.sqrt(b*b - 4 * a * c))/(2 * a)

                interpolated = points[i] + root * point_diff

                return interpolated, i - 1, False

        # If all future points are less than the look ahead distance
        # Return the very last point
        return points[-1], len(points) - 1, False
