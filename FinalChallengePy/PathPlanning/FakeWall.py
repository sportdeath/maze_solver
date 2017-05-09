from FinalChallengePy.Utils.GeomUtils import GeomUtils

class FakeWall:

    @staticmethod
    def makeFakeWall(point, normal, direction, rangeMethod):
        # the perpendicular of the normal
        perpendicular = direction * GeomUtils.getPerpendicular(normal)

        # The angle of the perpendicular
        angle = GeomUtils.getAngle(perpendicular)

        # the distance to wall
        distance = rangeMethod(point, angle)

        # The point on the wall
        endPoint = point + distance * perpendicular
        
        return (point, endPoint)
