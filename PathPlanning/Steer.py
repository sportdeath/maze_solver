from CarConstants import *
import numpy as np

MIN_RADIUS = np.tan(MAX_STEERING_ANGLE)/CAR_LENGTH
POINT_SPACING = 0.1
ANGLE_SPACING = POINT_SPACING/MIN_RADIUS

class Steer:
    steerable = True

    #def __init__(self, init, goal, unoccupiedPoints):
    def __init__(self, init, goal):
        positionDifference = goal.getPosition() - init.getPosition()

        # Which way do we turn at the start?
        self.initSide = Steer.sign(
                np.cross(
                    positionDifference, 
                    init.getOrientation()))

        # Compute the center of the circle tangent
        # to the init state
        initCenterNorm = -self.initSide * np.array([-init.getOrientation()[1], init.getOrientation()[0]])
        self.initCircleCenter = init.getPosition() + (initCenterNorm * MIN_RADIUS)

        # Check for steer-ability:
        if np.linalg.norm(goal.getPosition() - self.initCircleCenter) < MIN_RADIUS:
            # if the goal is within the radius
            # around the start, then we cannot
            # steer to it
            self.steerable = False
            return

        # Which way do we turn at the end?
        self.goalSide = Steer.sign(
                np.cross(
                    goal.getOrientation(), 
                    positionDifference))

        # Compute the center of the circle tangent
        # to the goal state
        goalCenterNorm = -self.goalSide * np.array([-goal.getOrientation()[1], goal.getOrientation()[0]])
        self.goalCircleCenter = goal.getPosition() + (goalCenterNorm * MIN_RADIUS)

        # Compute the norm that  points from the
        # center of the init circle to the center
        # of the goal circle
        circleNorm = self.goalCircleCenter - self.initCircleCenter
        circleDistance = np.linalg.norm(circleNorm)
        circleNorm /= circleDistance

        # Are we turning the same way?
        if self.initSide == self.goalSide:
            # If so, we are doing a U-Turn

            # A line connecting the tangent points
            # on both circles is perpendicular to the circle norm
            initTangentNorm = self.initSide * np.array([-circleNorm[1], circleNorm[0]])
            goalTangentNorm = initTangentNorm
        else:
            # If not, we are doing an S-Turn
            if circleDistance < 2 * MIN_RADIUS:
                self.steerable = False
                return

            # This is the angle relative to the circle norm
            # that the tangent point is at.
            turningAngle = np.arccos(2 * MIN_RADIUS/circleDistance)

            initTangentNorm = Steer.rotateVector(circleNorm, self.initSide * turningAngle)
            goalTangentNorm = -initTangentNorm

        # These are the tangent points
        # on both circles
        self.initTangentPoint = self.initCircleCenter + MIN_RADIUS * initTangentNorm
        self.goalTangentPoint = self.goalCircleCenter + MIN_RADIUS * goalTangentNorm

        # The line which runs between both circles
        # is given by the path norm and length
        self.lineNorm = self.goalTangentPoint - self.initTangentPoint
        self.lineLength = np.linalg.norm(self.lineNorm)
        self.lineNorm /= self.lineLength

        # These are the norms to the start and end positions
        initNorm = (init.getPosition() - self.initCircleCenter)/MIN_RADIUS
        goalNorm = (goal.getPosition() - self.goalCircleCenter)/MIN_RADIUS

        # Compute the angle from the start to the end
        self.initAngle = self.getAngleBetweenVectors(
                initNorm, 
                initTangentNorm, 
                self.initSide)
        self.goalAngle = Steer.getAngleBetweenVectors(
                goalTangentNorm,
                goalNorm, 
                self.goalSide)

        # Compute the starting angle of the turn
        self.initStartAngle = self.getAngle(initNorm)
        self.goalStartAngle = self.getAngle(goalTangentNorm)

        # Use ray marching with robot model to check for collisions
        # self.steerable = self.isCollisionFree()

    @staticmethod
    def rotateVector(vector, angle):
        sinAngle = np.sin(angle)
        cosAngle = np.cos(angle)
        rotationMatrix = np.array(
                [[cosAngle, -sinAngle],
                 [sinAngle, cosAngle]])
        return np.dot(rotationMatrix, vector)

    @staticmethod
    def sign(value):
        return (2 * (value > 0)) - 1

    @staticmethod
    def getAngle(vector):
        return np.arccos(vector[1]) * Steer.sign(vector[0])

    @staticmethod
    def getAngleBetweenVectors(init, goal, direction):
        initAngle = direction * Steer.getAngle(init)
        goalAngle = direction * Steer.getAngle(goal)
        if goalAngle < initAngle:
            goalAngle += 2 * np.pi
        return goalAngle - initAngle

    def getPoints(self):
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
            
        return points

    @staticmethod
    def getPointsOnCircle(center, startAngle, totalAngle, direction):
        points = []

        angleOffset = 0.
        while angleOffset < totalAngle:
            angle = (direction * angleOffset) + startAngle
            norm = MIN_RADIUS * np.array([np.sin(angle), np.cos(angle)])
            points.append(norm + center)
            angleOffset += ANGLE_SPACING

        return points

    @staticmethod
    def getPointsOnLine(start, norm, totalLength):
        points = []

        length = 0
        while length < totalLength:
            points.append(start + length * norm)
            length += POINT_SPACING

        return points

    def isSteerable(self):
        # Not steerable the goal point
        # is inside the intial point's circle
        # or if there are collisions
        return self.steerable

    def length(self):
        length = 0
        length += MIN_RADIUS * self.initAngle
        length += self.lineLength
        length += MIN_RADIUS * self.goalAngle
        return length

    def isCollisionFree(self):
        # TODO
        # This right now makes the assumption
        # that the car is a point.
        # It is also slow because it must generate
        # points and iterate over them.
        # This should instead use range_libc
        # 2 calls to range lib c can tell us if the entire
        # line contains intersections
        # several additional calls will give us turning info
        points = self.getPoints()
        for point in points:
            if point not in self.unoccupiedPoints:
                return False
        return True


# using range libc?

# send rays from the very back wheels of the car for straight line segment
# if distance is less than length, from there, to point with car wheels at front
# than there is intersection

# if its turning than draw parallelogram

# assume no obstacles are less wide than the car
