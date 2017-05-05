import cv2
from CoordinateTransformations import CoordinateTransformations
import rospy
import numpy as np
from CameraSettings import *

class CircleThreshold:
    HSV_MIN = np.array([6, 90, 160])
    HSV_MAX = np.array([15, 255, 255])

    HSV_MIN_ORANGE = np.array([2, 182, 133])
    HSV_MAX_ORANGE = np.array([7, 254, 225])

    HSV_MIN_GREEN = np.array([74, 73, 39])
    HSV_MAX_GREEN = np.array([109, 185, 75])

    RANDOM_SAMPLE_SIZE = 50

    def __init__(self, lookAheadDistance):
        self.lookAheadDistance = lookAheadDistance  #lookahead distance in cm
        self.mask = None

        self.CoordinateTransformations = CoordinateTransformations(\
            TRANSFORMATION_MATRIX, DISTANCE_FROM_BACK_WHEEL, DISTANCE_FROM_CENTER)

    def threshold(self, inputImage):
        hsvImage = cv2.cvtColor(inputImage, cv2.COLOR_BGR2HSV)
        self.mask = cv2.inRange(hsvImage, self.HSV_MIN_ORANGE, self.HSV_MAX_ORANGE)
        self.mask = cv2.erode(self.mask, None, iterations = 2)
        # Dilate
        self.mask = cv2.dilate(self.mask, None, iterations = 2)

    def findBestContour(self):
        _, contours, _ = cv2.findContours(self.mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        self.contourPoints = [point for contour in contours for point in contour]

    def transform(self):
        numPoints = len(self.contourPoints)
        self.circle = []
        if numPoints > 0:
            self.circle = [0]*self.RANDOM_SAMPLE_SIZE
            for i in range(self.RANDOM_SAMPLE_SIZE):
                index = np.random.randint(0,numPoints)
                pixelCoordinates = self.contourPoints[index]
                worldCoordinates = self.CoordinateTransformations.transformPixelsToWorld(pixelCoordinates[0])
                self.circle[i] = worldCoordinates

    def findGoalPoint(self, inputImage):
        self.threshold(inputImage)
        self.findBestContour()
        self.transform()
        closest = None
        for (x, y) in self.circle:
            if closest: 
                if abs(np.linalg.norm((x, y))-self.lookAheadDistance) < abs(np.linalg.norm(closest)-self.lookAheadDistance):
                    closest = (x, y)
            else:
                closest = (x, y)
        
        return closest

    def getDistance(self, inputImage):
        self.threshold(inputImage)
        self.findBestContour()
        pixelPoint = max(self.contourPoints, key = lambda x: x[0][1])[0]
        worldCoord = self.CoordinateTransformations.transformPixelsToWorld(pixelPoint)
        return pixelPoint, worldCoord

if __name__ == "__main__":
    c = CircleThreshold(150)

    img = cv2.imread("/home/racecar/racecar-ws/src/FinalChallenge/Vision/images/x50y200r.png")

    pixelPoint, worldCoord = c.getDistance(img)

    print pixelPoint
    print worldCoord

    cv2.circle(img, (int(pixelPoint[0]), int(pixelPoint[1])), 4, (0, 255, 0), -1)

    cv2.imshow("image", img)
    cv2.waitKey(0)

    # The code below can be used to pick max and min hsv values.
    # Just uncomment and click points in the area you want to sample.

    # values = []
    # hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # def get_hsv(event, x, y, flags, param):
    #     global values, hsv
        
    #     if event == cv2.EVENT_LBUTTONDOWN:
	   #      values.append(hsv[y, x])

    # cv2.namedWindow("image")
    # cv2.setMouseCallback("image", get_hsv)


    # while True:
    # # display the image and wait for a keypress
    #     cv2.imshow("image", img)
    #     key = cv2.waitKey(1) & 0xFF    

    #     if key == ord("c"):
    #         break

    # print values
    # print "min: "
    # print np.min(values, axis = 0).tolist()
    # print "max: "
    # print np.max(values, axis = 0).tolist()
    # cv2.destroyAllWindows()


