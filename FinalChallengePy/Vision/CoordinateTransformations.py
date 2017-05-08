import numpy as np

from FinalChallengePy.Vision.Constants import *

scalingFactor = np.loadtxt(SCALING_FACTOR_FILE)
rotationMatrix = np.loadtxt(ROTATION_MATRIX_FILE)
rotationMatrixInv = np.loadtxt(ROTATION_MATRIX_INV_FILE)
translationVector = np.loadtxt(TRANSLATION_VECTOR_FILE)
cameraMatrix = np.loadtxt(CAMERA_MATRIX_FILE)
cameraMatrixInv = np.loadtxt(CAMERA_MATRIX_INV_FILE)

class CoordinateTransformations:
    @staticmethod
    def pixelsToLocal(pixels):
        pixelsAffine = np.array([pixels[0], pixels[1], 1.])
        pinHoleCoordinates = scalingFactor * np.dot(cameraMatrixInv, pixelsAffine)
        worldAffine = np.dot(rotationMatrixInv, pinHoleCoordinates - translationVector)
        return np.array([worldAffine[0], worldAffine[1]])
       
    @staticmethod
    def localToPixels(world):
        worldAffine = np.array([world[0], world[1], 1.])
        pinHoleCoordinates = np.dot(rotationMatrix, world) + translationVector
        pixelsAffine = np.dot(cameraMatrix, pinHoleCoordinates)/scalingFactor
        return np.array([pixelsAffine[0], pixelsAffine[1]])
