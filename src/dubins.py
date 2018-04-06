import numpy as np

from maze_solver.utils.geom_utils import GeomUtils

from FinalChallengePy.CarConstants import *

POINT_SPACING = (CAR_FORWARD_LENGTH + CAR_REAR_LENGTH)/3.
ANGLE_SPACING = POINT_SPACING/MIN_RADIUS
DIST_FROM_CIRCLE_AXIS = (MIN_RADIUS - CAR_WIDTH/2.) 
CIRCLE_WIDTH_ADJUSTMENT = DIST_FROM_CIRCLE_AXIS- np.cos(ANGLE_SPACING)/DIST_FROM_CIRCLE_AXIS + 0.25
print("Circle width adjustment", CIRCLE_WIDTH_ADJUSTMENT)

class Dubins:

    def __init__(self, max_steering_angle, car_axle_dist):
        self.MIN_RADIUS = np.tan(max_steering_angle)/car_axle_dist

    def find_curve(self, init, goal):
        """
        Args:
            init: (x, y, theta)
            goal: (x, y, theta)

        Return:
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

        # Use ray marching with robot model to check for collisions
        self.exists = True
        self.steerable = self.isCollisionFree(rangeMethod)

    def get_circle_center(self, pos, vec, side):
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

    def computeAngles(
            self, 
            init_pos, 
            init_vec, 
            init_side, 
            goal_pos, 
            goal_vec,
            goal_side):

        # Compute init and goal circle centers
        init_cc = self.get_circle_center(init_pos, init_vec, init_side)
        goal_cc = self.get_circle_center(goal_pos, goal_vec, goal_side)

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


    def getPoints(self, oriented=False, goalExtension=0.):
        points = []

        # Draw points along init arc
        points += Steer.getPointsOnCircle(
                self.initCircleCenter,
                self.initStartAngle,
                self.initAngle,
                self.initSide)

        # Draw points on line
        points += Steer.getPointsOnLine(
                self.initTangentPoint,
                self.lineNorm,
                self.lineLength)

        # Draw points along goal arc
        points += Steer.getPointsOnCircle(
                self.goalCircleCenter,
                self.goalStartAngle,
                self.goalAngle,
                self.goalSide)
        
        if self.backwards:
            points.reverse()

        extension = 0
        while extension < goalExtension:
            offset = extension * self.goal.getOrientation()
            if self.backwards:
                offset *= -1.
            points.append(offset + self.goal.getPosition())
            extension += POINT_SPACING
        # Make sure to add the last point
        offset = goalExtension * self.goal.getOrientation()
        if self.backwards:
            offset *= -1.
        points.append(offset + self.goal.getPosition())

        if oriented:
            return (points, self.backwards)

        return points

    @staticmethod
    def getPointsOnCircle(center, startAngle, totalAngle, direction):
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
    def getPointsAndNormsOnCircle(center, startAngle, totalAngle, direction):
        pointsAndNorms = []

        angleOffset = 0.
        while angleOffset < totalAngle:
            angle = (-direction * angleOffset) + startAngle
            axle = GeomUtils.getVector(angle)
            pointsAndNorms.append((MIN_RADIUS * axle + center, direction * GeomUtils.getPerpendicular(axle)))
            angleOffset += ANGLE_SPACING
        angleOffset = totalAngle
        angle = (-direction * angleOffset) + startAngle
        axle = GeomUtils.getVector(angle)
        pointsAndNorms.append((MIN_RADIUS * axle + center, direction * GeomUtils.getPerpendicular(axle)))

        return pointsAndNorms

    def getPointsAndNorms(self):
        initPointsAndNorms = Steer.getPointsAndNormsOnCircle(
                self.initCircleCenter,
                self.initStartAngle,
                self.initAngle,
                self.initSide)

        goalPointsAndNorms = Steer.getPointsAndNormsOnCircle(
                self.goalCircleCenter,
                self.goalStartAngle,
                self.goalAngle,
                self.goalSide)

        return initPointsAndNorms + goalPointsAndNorms

    @staticmethod
    def getPointsOnLine(start, norm, totalLength):
        points = []

        length = 0
        while length < totalLength:
            points.append(start + length * norm)
            length += POINT_SPACING

        return points

    @staticmethod
    def intersectCirclePoints(line, center, radius):
        lineDifference = line[1] - line[0]
        circleDifference = line[0] - center

        a = np.dot(lineDifference, lineDifference)
        b = 2.*np.dot(circleDifference, lineDifference)
        c = np.dot(circleDifference, circleDifference) - \
                radius * radius

        innerRoot = b*b - 4.*a*c
        if innerRoot < 0:
            return []

        denominator = 2.*a

        root = np.sqrt(innerRoot)/denominator
        constant = -b/denominator

        ts = [constant + root, constant - root]

        points = []
        for t in ts:
            if 0 <= t <= 1:
                point = t * lineDifference + line[0]
                points.append(point)

        return points
    def getLength(self):
        length = 0
        length += MIN_RADIUS * self.initAngle
        length += self.lineLength
        length += MIN_RADIUS * self.goalAngle
        return length
