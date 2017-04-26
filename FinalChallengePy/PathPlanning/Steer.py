from FinalChallengePy.CarConstants import *
import numpy as np

MIN_RADIUS = np.tan(MAX_STEERING_ANGLE)/CAR_AXLE_DISTANCE
POINT_SPACING = CAR_FORWARD_LENGTH + CAR_REAR_LENGTH
ANGLE_SPACING = POINT_SPACING/MIN_RADIUS
DIST_FROM_CIRCLE_AXIS = (MIN_RADIUS - CAR_WIDTH/2.) 
CIRCLE_WIDTH_ADJUSTMENT = DIST_FROM_CIRCLE_AXIS- np.cos(ANGLE_SPACING)/DIST_FROM_CIRCLE_AXIS

class Steer:
    exists = False
    steerable = False
    backwards = False

    def __init__(self, init, goal, rangeMethod):
        if goal.isBackwards():
            init, goal=goal, init
            self.backwards = True

        self.rangeMethod = rangeMethod

        positionDifference = goal.getPosition() - init.getPosition()

        if np.dot(positionDifference,positionDifference) == 0:
            return

        # Which way do we turn at the start?
        self.initSide = Steer.sign(
                np.cross(
                    positionDifference, 
                    init.getOrientation()))

        # Compute the center of the circle tangent
        # to the init state
        initCenterNorm = self.initSide * Steer.getPerpendicular(init.getOrientation())
        self.initCircleCenter = init.getPosition() + (initCenterNorm * MIN_RADIUS)

        # Check for steer-ability:
        if np.linalg.norm(goal.getPosition() - self.initCircleCenter) < MIN_RADIUS:
            # if the goal is within the radius
            # around the start, then we cannot
            # steer to it
            return

        # Which way do we turn at the end?
        self.goalSide = Steer.sign(
                np.cross(
                    goal.getOrientation(), 
                    positionDifference))

        # Compute the center of the circle tangent
        # to the goal state
        goalCenterNorm = self.goalSide * Steer.getPerpendicular(goal.getOrientation())
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
            initTangentNorm = -self.initSide * Steer.getPerpendicular(circleNorm)
            goalTangentNorm = initTangentNorm
        else:
            # If not, we are doing an S-Turn
            if circleDistance < 2 * MIN_RADIUS:
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
        self.exists = True
        self.steerable = self.isCollisionFree()

    @staticmethod
    def getPerpendicular(norm):
        return np.array([norm[1], -norm[0]])

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
        return np.arccos(np.clip(vector[1],-1,1)) * Steer.sign(vector[0])

    @staticmethod
    def getAngleBetweenVectors(init, goal, direction):
        initAngle = direction * Steer.getAngle(init)
        goalAngle = direction * Steer.getAngle(goal)
        if goalAngle < initAngle:
            goalAngle += 2 * np.pi
        return goalAngle - initAngle

    def getPoints(self, oriented = False):
        points = []

        # points.append(self.init.getPosition())

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

        if oriented:
            for i in xrange(len(points)):
                points[i] = (points[i],self.backwards)
        
        if self.backwards:
            points.reverse()

        return points

    @staticmethod
    def getPointsOnCircle(center, startAngle, totalAngle, direction):
        points = []

        angleOffset = 0.
        while angleOffset < totalAngle:
            angle = (direction * angleOffset) + startAngle
            axle = MIN_RADIUS * np.array([np.sin(angle), np.cos(angle)])
            points.append(axle + center)
            angleOffset += ANGLE_SPACING

        return points

    @staticmethod
    def getPointsAndNormsOnCircle(center, startAngle, totalAngle, direction):
        pointsAndNorms = []

        angleOffset = 0.
        while angleOffset < totalAngle:
            angle = (direction * angleOffset) + startAngle
            axle = np.array([np.sin(angle), np.cos(angle)])
            pointsAndNorms.append((MIN_RADIUS * axle + center, Steer.getPerpendicular(axle)))
            angleOffset += ANGLE_SPACING

        return pointsAndNorms

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

    def getLength(self):
        length = 0
        length += MIN_RADIUS * self.initAngle
        length += self.lineLength
        length += MIN_RADIUS * self.goalAngle
        return length

    def testCollisions(self, center, width, length, norm):
        verticalOffset = -norm * CAR_REAR_LENGTH

        horizontalOffset = Steer.getPerpendicular(norm) * width/2.
        angle = Steer.getAngle(norm)

        minLength = CAR_REAR_LENGTH + length + CAR_FORWARD_LENGTH

        for side in [-1., 1.]:
            backWheel = (side * horizontalOffset) + verticalOffset + center
        
            distanceToObstacle = self.rangeMethod(backWheel, angle)
            if distanceToObstacle < minLength:
                return False

        return True

    def isCollisionFree(self):
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

        for (point,norm) in (goalPointsAndNorms + initPointsAndNorms):
            isFree = self.testCollisions(
                    point,
                    CAR_WIDTH + CIRCLE_WIDTH_ADJUSTMENT,
                    0.,
                    norm)
            if not isFree:
                return False

        # To check intersections along the line
        # Have points at the car's left and right
        # back wheel and propagate them forward
        return self.testCollisions(
                self.initTangentPoint,
                CAR_WIDTH,
                self.lineLength,
                self.lineNorm)
