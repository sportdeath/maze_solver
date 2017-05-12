import numpy as np

class GeomUtils:
    @staticmethod
    def sign(value):
        return (2 * (value > 0)) - 1

    @staticmethod
    def getAngle(vector):
        return np.arccos(np.clip(vector[1],-1,1)) * GeomUtils.sign(vector[0])

    @staticmethod
    def getVector(angle):
        return np.array([np.sin(angle), np.cos(angle)])

    @staticmethod
    def getAngleBetweenVectors(init, goal, direction):
        initAngle = direction * GeomUtils.getAngle(init)
        goalAngle = direction * GeomUtils.getAngle(goal)
        if goalAngle < initAngle:
            goalAngle += 2 * np.pi
        return goalAngle - initAngle

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
