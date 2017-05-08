import numpy as np
import cv2

class CoordinateTransformations:

    def __init__(self, transformationMatrix, distanceToBackWheel, distanceToCenter):
        self.transformationMatrix = transformationMatrix
        self.inverseTransformationMatrix = np.linalg.inv(transformationMatrix)
        # in CM
        self.distanceToBackWheel = distanceToBackWheel
        self.distanceToCenter = distanceToCenter

    # (0,0) is the center of the back wheel axle
    # measurement is in CM
    def transformPixelsToWorld(self, pixelCoordinates):
        affinePixelCoordinates = (pixelCoordinates[0], pixelCoordinates[1], 1)
        affineWorldCoordinates = np.matmul(self.transformationMatrix, affinePixelCoordinates)

        worldCoordinates = (
                affineWorldCoordinates[0]/affineWorldCoordinates[2] + self.distanceToCenter, 
                -affineWorldCoordinates[1]/affineWorldCoordinates[2] + self.distanceToBackWheel
                )

        # returns measurement in cm
        return worldCoordinates

    def transformWorldToPixels(self, worldCoordinates):
        affineWorldCoordinates = (worldCoordinates[0] - self.distanceToCenter, -(worldCoordinates[1] - self.distanceToBackWheel), 1)

        affinePixelCoordinates = np.matmul(self.inverseTransformationMatrix, affineWorldCoordinates)

        pixelCoordinates = (int(affinePixelCoordinates[0]/affinePixelCoordinates[2]), int(affinePixelCoordinates[1]/affinePixelCoordinates[2]))

        # returns measurement in pixels
        return pixelCoordinates

    def displayCoordinatesOnWorld(self, image, gridDim, gridSize):
        # copy image
        outputImage = image.copy()
        h,  w = outputImage.shape[:2]

        # make a grid with points at 1m
        grid = []
        for i in range(-gridSize, gridSize+1):
            for j in range(-gridSize, gridSize+1):
                grid.append((i*gridDim, j*gridDim))

        pixelGrid = []
        # convert to worldCoordinates
        for point in grid:
            pixelPoint = self.transformWorldToPixels(point)
            cv2.circle(outputImage,pixelPoint,1,(0,0,255),thickness=1)

        return outputImage
