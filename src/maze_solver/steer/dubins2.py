import numpy as np

from maze_solver.utils.geom_utils import GeomUtils

from FinalChallengePy.CarConstants import *

POINT_SPACING = (CAR_FORWARD_LENGTH + CAR_REAR_LENGTH)/3.
ANGLE_SPACING = POINT_SPACING/MIN_RADIUS
DIST_FROM_CIRCLE_AXIS = (MIN_RADIUS - CAR_WIDTH/2.) 
CIRCLE_WIDTH_ADJUSTMENT = DIST_FROM_CIRCLE_AXIS- np.cos(ANGLE_SPACING)/DIST_FROM_CIRCLE_AXIS + 0.25
MIN_RADIUS = np.tan(max_steering_angle)/car_axle_dist
print("Circle width adjustment", CIRCLE_WIDTH_ADJUSTMENT)

class DubinsPath:

    def __init__(self, init, goal):
        """
        Args:
            init: (x, y, theta)
            goal: (x, y, theta)
        """

        if np.all(init == goal):
            return

        # Get constant
        init_pos = init[:2]
        goal_pos = goal[:2]
        init_vec = GeomUtils.get_vector(init[2])
        goal_vec = GeomUtils.get_vector(goal[2])

        pos_diff = goal_pos - init_pos

        # Which way do we turn at the start?
        #  1 = turn right
        # -1 = turn left
        init_side = np.sign(np.cross(pos_diff, init_vec))

        # Which way do we turn at the end?
        goal_side = np.sign(np.cross(goal_vec, pos_diff))

        # Try to turn at this side
        initNorm, goalTangentNorm, badSTurn = self.computeAngles(init, goal)

        if badSTurn:
            return

        # If we loopty-loop swap
        if self.initAngle > np.pi*3./2.:
            self.initSide = -self.initSide
            initNorm, goalTangentNorm, badSTurn = self.computeAngles(init, goal)
        elif self.goalAngle > np.pi*3./2.:
            self.goalSide = -self.goalSide
            initNorm, goalTangentNorm, badSTurn = self.computeAngles(init, goal)

        if badSTurn:
            return

        # Compute the starting angle of the turn
        self.initStartAngle = GeomUtils.getAngle(initNorm)
        self.goalStartAngle = GeomUtils.getAngle(goalTangentNorm)

    def computeAngles(
            self, 
            init_pos, 
            init_vec, 
            init_side, 
            goal_pos, 
            goal_vec,
            goal_side):

        # Compute init and goal circle centers
        init_cc = DubinsPath.circle_center(init_pos, init_vec, init_side)
        goal_cc = DubinsPath.circle_center(goal_pos, goal_vec, goal_side)

        # Compute the norm that  points from the
        # center of the init circle to the center
        # of the goal circle
        circle_diff = goal_cc - init_cc
        circle_dist = np.linalg.norm(circle_norm)
        circle_norm = circle_diff/circle_dist

        # Are we turning the same way?
        if init_side == goal_side:
            # If so, we are doing a U-Turn

            # The vectors pointing from the center of
            # each to the point where a line connects them
            # is perpendicular to the circle norm.
            init_tangent_norm = -init_side * GeomUtils.get_perpendicular(circle_norm)
            goal_tangent_norm = init_tangent_norm
        else:
            # If not, we are doing an S-Turn
            if circle_dist < 2 * self.MIN_RADIUS:
                return False

            # This is the angle relative to the circle norm
            # that the tangent point is at.
            turning_angle = np.arccos(2 * self.MIN_RADIUS/circle_dist)

            init_tangent_norm = GeomUtils.rotate_vector(circleNorm, init_side * turning_angle)
            goal_tangent_norm = -init_tangent_norm

        # These are the tangent points on both circles
        init_tangent_point = init_cc + self.MIN_RADIUS * init_tangent_norm
        goal_tangent_point = goal_cc + self.MIN_RADIUS * goal_tangent_norm

        # The line which runs between both circles
        # is given by the path norm and length
        line_diff = goal_tangent_point - init_tangent_point
        line_length = np.linalg.norm(line_diff)
        line_norm = line_diff/line_length

        return False, init_cc, init_start_angle, init_angle, goal_side init_tangent_point, line_diff, goal_cc, goal_start_angle, goal_angle, goal_side

        # These are the norms to the start and end positions
        initNorm = (init.getPosition() - self.initCircleCenter)/MIN_RADIUS
        goalNorm = (goal.getPosition() - self.goalCircleCenter)/MIN_RADIUS

        # Compute the angle from the start to the end
        self.initAngle = GeomUtils.getAngleBetweenVectors(
                initNorm, 
                initTangentNorm, 
                self.initSide)
        self.goalAngle = GeomUtils.getAngleBetweenVectors(
                goalTangentNorm,
                goalNorm, 
                self.goalSide)

        return (initNorm, goalTangentNorm, False)

    @staticmethod
    def circle_center(self, pos, vec, side):
        """
        Compute the center of the circle that
        is tangent to the pose on the appropriate side.

        Args:
            pos: The position of the pose.
            vec: The orientation of the pose as a unit vector.
            side: The side the circle is on. 
                1 = Right, -1 = Left.
        """

        center_norm = side * GeomUtils.get_perpendicular(vec)

        return pos + center_norm * self.MIN_RADIUS

    def length(self):
        length = 0
        length += MIN_RADIUS * self.init_angle
        length += self.line_length
        length += MIN_RADIUS * self.goal_angle

        return length

    def sample(self, sample_width):
        points =  (
                # Sample points along the init arc
                DubinsPath.sample_circle(
                    sample_width,
                    self.init_cc,
                    self.init_start_angle,
                    self.init_angle,
                    self.init_side),
                # Sample points along the line
                DubinsPath.sample_line(
                    sample_width,
                    self.line_start_point,
                    self.line_diff),
                # Sample points along the goal arc
                DubinsPath.sample_circle(
                    sample_width,
                    self.goal_cc,
                    self.goal_start_angle,
                    self.goal_angle,
                    self.goal_side))

        points = np.concatenate(points, axis=0)

        return points

    @staticmethod
    def sample_circle(sample_width, center, startAngle, totalAngle, direction):
        points = []

        angleOffset = 0.
        while angleOffset < totalAngle:
            angle = (-direction * angleOffset) + startAngle
            axle = MIN_RADIUS * GeomUtils.getVector(angle)
            points.append(axle + center)
            angleOffset += ANGLE_SPACING
        angleOffset = totalAngle
        angle = (-direction * angleOffset) + startAngle
        axle = MIN_RADIUS * GeomUtils.getVector(angle)
        points.append(axle + center)

        return points

    @staticmethod
    def sample_line(sample_width, start, norm, totalLength):
        points = []

        length = 0
        while length < totalLength:
            points.append(start + length * norm)
            length += POINT_SPACING

        return points

