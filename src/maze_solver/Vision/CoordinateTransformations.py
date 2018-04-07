import numpy as np

from FinalChallengePy.Vision.Constants import *

transformationMatrix = np.loadtxt(TRANSFORMATION_MATRIX_FILE)
transformationMatrixInv = np.loadtxt(TRANSFORMATION_MATRIX_INV_FILE)
cameraMatrix = np.loadtxt(CAMERA_MATRIX_FILE)
cameraMatrixInv = np.loadtxt(CAMERA_MATRIX_INV_FILE)

class CoordinateTransformations:
    @staticmethod
    def pixelsToLocal(pixels):
        pixelsAffine = np.array([pixels[0], pixels[1], 1.])
        pinHoleCoordinates = np.dot(cameraMatrixInv, pixelsAffine)
        worldAffine = np.dot(transformationMatrixInv, pinHoleCoordinates)
        return np.array([worldAffine[0], worldAffine[1]])/worldAffine[2]
       
    @staticmethod
    def localToPixels(world):
        worldAffine = np.array([world[0], world[1], 1.])
        pinHoleCoordinates = np.dot(transformationMatrix, worldAffine)
        pixelsAffine = np.dot(cameraMatrix, pinHoleCoordinates)
        return np.array([pixelsAffine[0], pixelsAffine[1]])/pixelsAffine[2]
