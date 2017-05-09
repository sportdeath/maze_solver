import numpy as np

from FinalChallengePy.Utils.GeomUtils import GeomUtils

from FinalChallengePy.CarConstants import *

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
        self.goal = goal
        if goal.isBackwards():
            init, goal=goal, init
            self.backwards = True

        positionDifference = goal.getPosition() - init.getPosition()

        if np.dot(positionDifference,positionDifference) == 0:
            return

        # Which way do we turn at the start?
        self.initSide = GeomUtils.sign(
                np.cross(
                    positionDifference, 
                    init.getOrientation()))

        # Which way do we turn at the end?
        self.goalSide = GeomUtils.sign(
                np.cross(
                    goal.getOrientation(), 
                    positionDifference))

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

    @staticmethod
    def getCircleCenter(state, side):
        # Compute the center of the circle tangent
        # to the init state
        centerNorm = side * GeomUtils.getPerpendicular(state.getOrientation())
        return state.getPosition() + (centerNorm * MIN_RADIUS)

    def computeAngles(self, init, goal):
        self.initCircleCenter = Steer.getCircleCenter(init, self.initSide)
        self.goalCircleCenter = Steer.getCircleCenter(goal, self.goalSide)

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
            initTangentNorm = -self.initSide * GeomUtils.getPerpendicular(circleNorm)
            goalTangentNorm = initTangentNorm
        else:
            # If not, we are doing an S-Turn
            if circleDistance < 2 * MIN_RADIUS:
                return (None, None, True)

            # This is the angle relative to the circle norm
            # that the tangent point is at.
            turningAngle = np.arccos(2 * MIN_RADIUS/circleDistance)

            initTangentNorm = GeomUtils.rotateVector(circleNorm, self.initSide * turningAngle)
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
            angle = (direction * angleOffset) + startAngle
            axle = MIN_RADIUS * np.array([np.sin(angle), np.cos(angle)])
            points.append(axle + center)
            angleOffset += ANGLE_SPACING
        angleOffset = totalAngle
        angle = (direction * angleOffset) + startAngle
        axle = MIN_RADIUS * np.array([np.sin(angle), np.cos(angle)])
        points.append(axle + center)

        return points

    @staticmethod
    def getPointsAndNormsOnCircle(center, startAngle, totalAngle, direction):
        pointsAndNorms = []

        angleOffset = 0.
        while angleOffset < totalAngle:
            angle = (direction * angleOffset) + startAngle
            axle = np.array([np.sin(angle), np.cos(angle)])
            pointsAndNorms.append((MIN_RADIUS * axle + center, GeomUtils.getPerpendicular(axle)))
            angleOffset += ANGLE_SPACING
        angleOffset = totalAngle
        angle = (direction * angleOffset) + startAngle
        axle = np.array([np.sin(angle), np.cos(angle)])
        pointsAndNorms.append((MIN_RADIUS * axle + center, GeomUtils.getPerpendicular(axle)))

        return pointsAndNorms

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

    @staticmethod
    def intersectsCircle(
            line, 
            center, 
            radius, 
            startAngle, 
            totalAngle, 
            direction):
        points = Steer.intersectCirclePoints(line, center, radius)

        for point in points:
            relative = point - center
            norm = relative/np.linalg.norm(relative)
            angle = GeomUtils.getAngle(norm)

            relativeAngle = np.mod(direction * (angle - startAngle), 2.*np.pi)
            if relativeAngle < totalAngle:
                return True
            
        return False

    @staticmethod
    def intersectsLine(line0, line1):
        difference0 = line0[1] - line0[0]
        difference1 = line1[1] - line1[0]

        startDifference = line1[0] - line0[0]

        denominator = np.cross(difference0, difference1)
        numerator0 = np.cross(startDifference, difference1)

        if denominator == 0:
            if numerator0 == 0:
                return True
            else:
                return False

        numerator1 = np.cross(startDifference, difference0)

        t0 = numerator0/denominator
        t1 = numerator1/denominator

        return 0 <= t0 <= 1 and 0 <= t1 <= 1

    # line is a start and end point
    # like whats generated by Fake Wall
    def intersects(self, line):
        # Check intersection with start and end circles
        intersectsLine = Steer.intersectsLine(line, (self.initTangentPoint, self.goalTangentPoint))

        if intersectsLine:
            return True

        intersectsInit = Steer.intersectsCircle(
                line,
                self.initCircleCenter,
                MIN_RADIUS,
                self.initStartAngle,
                self.initAngle,
                self.initSide)

        if intersectsInit:
            return True

        intersectsGoal = Steer.intersectsCircle(
                line,
                self.goalCircleCenter,
                MIN_RADIUS,
                self.goalStartAngle,
                self.goalAngle,
                self.goalSide)

        if intersectsGoal:
            return True
        
        return False

    def isSteerable(self, fakeWall = None):
        # Not steerable the goal point
        # is inside the intial point's circle
        # or if there are collisions
        if fakeWall is not None:
            return self.steerable and (not self.intersects(fakeWall))
        else:
            return self.steerable

    def getLength(self):
        length = 0
        length += MIN_RADIUS * self.initAngle
        length += self.lineLength
        length += MIN_RADIUS * self.goalAngle
        return length

    def testCollisions(self, center, width, length, norm, rangeMethod):
        verticalOffset = -norm * CAR_REAR_LENGTH

        horizontalOffset = GeomUtils.getPerpendicular(norm) * width/2.
        angle = GeomUtils.getAngle(norm)

        minLength = CAR_REAR_LENGTH + length + CAR_FORWARD_LENGTH

        for side in [-1., 1.]:
            backWheel = (side * horizontalOffset) + verticalOffset + center
        
            distanceToObstacle = rangeMethod(backWheel, angle)
            if distanceToObstacle < minLength:
                return False

        return True

    def getGoalState(self):
        return self.goal

    def isCollisionFree(self, rangeMethod):
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
                    norm,
                    rangeMethod)
            if not isFree:
                return False

        # To check intersections along the line
        # Have points at the car's left and right
        # back wheel and propagate them forward
        return self.testCollisions(
                self.initTangentPoint,
                CAR_WIDTH,
                self.lineLength,
                self.lineNorm,
                rangeMethod)
