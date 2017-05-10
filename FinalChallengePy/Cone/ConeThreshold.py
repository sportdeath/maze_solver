import cv2
import numpy as np
import rospy

from geometry_msgs.msg import Point

'''
Adapted from tutorial:
http://www.pyimagesearch.com/2014/08/04/opencv-python-color-detection/
'''

class ConeThreshold:
    ASPECT_MIN = 0.5
    ASPECT_MAX = 1
    exists = True

    def __init__(self, inputImage, thresholdMin, thresholdMax):
        self.inputImage = inputImage
        self.hsvMin = thresholdMin
        self.hsvMax = thresholdMax

        # Convert to HSV
        hsvImage = cv2.cvtColor(self.inputImage, cv2.COLOR_BGR2HSV)

        # Get masks
        self.mask = self.getMask(hsvImage)

        # Get Contours
        self.contours = self.getContours(self.mask)
        # filter contours to select best
        self.bestContour = None
        self.bestHeight = None
        self.bestWidth = None
        self.bestxTop = None
        self.bestyTop = None
        self.filterContours()
        
        if self.bestContour is None:
            self.exists = False

    def getMask(self, hsvImage):
        # Threshold
        mask = cv2.inRange(hsvImage, self.hsvMin, self.hsvMax)
        # Erode
        mask = cv2.erode(mask, None, iterations = 1)
        # Dilate
        mask = cv2.dilate(mask, None, iterations = 3)
        return mask
    
    def doesExist(self):
        return self.exists

    def getContours(self, mask):
        _, contours ,_ = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        return contours

    def getReferenceImage(self):
        return self.REFERENCE

    def getInputImage(self):
        return self.inputImage

    def getMaskImage(self):
        return self.mask

    def getMaskedImage(self):
    	# make a new image that is the original image only in the mask, and black everywhere else
        return cv2.bitwise_and(self.inputImage, self.inputImage, mask = self.mask)

    def getMatchedImage(self):
        for contour in self.contours:
            xTop, yTop, width, height = cv2.boundingRect(contour)

            matchedImage = self.inputImage.copy()

            refHeight, refWidth =  self.REFERENCE.shape[:2]

            referenceScaled = cv2.resize(\
                    self.REFERENCE, \
                    (width, height))

            matchedImage[ \
                    yTop:yTop + referenceScaled.shape[0],\
                    xTop:xTop + referenceScaled.shape[1]]\
                    = referenceScaled

            return matchedImage

    def filterContours(self):
        self.filteredContours = []
        bestArea = 0
        self.bestWidth = None

        for i in range(len(self.contours)):
            contour = self.contours[i]
            xTop, yTop, width, height = cv2.boundingRect(contour)
            aspectRatio = width/float(height)

            if (self.ASPECT_MIN < aspectRatio < self.ASPECT_MAX):
                self.filteredContours.append(contour)
                area = width * height

                if area  >= bestArea:
                    bestArea = area
                    self.bestContour = contour
                    self.bestWidth = width
                    self.bestHeight = height
                    self.bestxTop = xTop
                    self.bestyTop = yTop

    def getWidth(self):
        return self.bestWidth or -1

    def getHeight(self):
        return self.bestHeight or -1

    def getBottomCenterPoint(self):
        bottomCenterX = self.bestxTop + self.bestWidth/2
        bottomCenterY = self.bestyTop + self.bestHeight
        return np.array([bottomCenterX, bottomCenterY])

    def getBoundedImage(self, colorText):
        boundedImage = self.getMaskedImage()
        self.filterContours()

        for i in range(len(self.contours)):
            contour = self.contours[i]
            xTop, yTop, width, height = cv2.boundingRect(contour)
            aspectRatio = width/float(height)
            area = width * height

            if contour is self.bestContour:
                color = [0,255,0]
            else:
                color = [0,0,255]

            text = "C: " + colorText

            cv2.rectangle( \
                    boundedImage,\
                    (xTop, yTop), \
                    (xTop + width, yTop + height), \
                    color,\
                    2)

            cv2.putText( \
                    boundedImage,\
                    text,\
                    (xTop, yTop),\
                    cv2.FONT_HERSHEY_PLAIN,\
                    1,\
                    color,\
                    2)

            # mark the center point
            if self.doesExist():
                centerPoint = self.getBottomCenterPoint()
                centerPoint = (centerPoint[0], centerPoint[1])
                cv2.circle(boundedImage,centerPoint,2,[0,0,255],thickness=2)

        return boundedImage


# Testing code
if __name__=="__main__":
    HSV_MIN_ORANGE = np.array([2, 182, 133])
    HSV_MAX_ORANGE = np.array([7, 254, 225])

    HSV_MIN_GREEN = np.array([74, 73, 39])
    HSV_MAX_GREEN = np.array([109, 185, 85]) # might need to bump the last one up to like 85

    img = cv2.imread("/home/katy/racecar-ws/src/final_challenge/FinalChallengePy/Vision/images/redgreen8.png")

    redThreshold = ConeThreshold(img, HSV_MIN_ORANGE, HSV_MAX_ORANGE)
    greenThreshold = ConeThreshold(img, HSV_MIN_GREEN, HSV_MAX_GREEN)

    cv2.imshow("original", img)

    combined = cv2.add(redThreshold.getBoundedImage("red"), greenThreshold.getBoundedImage("green"))
    cv2.imshow("masked", combined)

    cv2.waitKey(0)

    cv2.destroyAllWindows()
