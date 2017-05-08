import cv2
import numpy as np
import rospy

'''
Adapted from tutorial:
http://www.pyimagesearch.com/2014/08/04/opencv-python-color-detection/
'''

# TODO: add methods to get the distance and mark new cone

class ConeThreshold:
    HSV_MIN_ORANGE = np.array([2, 182, 133])
    HSV_MAX_ORANGE = np.array([7, 254, 225])

    HSV_MIN_GREEN = np.array([74, 73, 39])
    HSV_MAX_GREEN = np.array([109, 185, 75]) # might need to bump the last one up to like 85

    MATCH_MAX = 0.5
    ASPECT_MIN = 0.5
    ASPECT_MAX = 1

    CAMERA_DISTORTION = np.asarray([-0.0203961,  -0.00518214, -0.00025931, -0.00115718,  0.01827989])
    CAMERA_MATRIX = np.matrix([[ 342.95689842,    0.      ,    300.94135395],
                     [   0.        ,  344.3969262,   179.72137603],
                     [   0.        ,    0.       ,     1.        ]])
    CAMERA_DIMENSIONS = (672, 376)
    DIST_TIMES_FOCAL_LENGTH_PX = 50/0.0068

    def __init__(self, inputImage, thresholdMin, thresholdMax):
        # undistort the image - TOO SLOW, commenting out for now
        self.imageHeight,  self.imageWidth = inputImage.shape[:2]
        #newCameraMatrix, roi = cv2.getOptimalNewCameraMatrix(\
        #        self.CAMERA_MATRIX, \
        #        self.CAMERA_DISTORTION, \
        #        self.CAMERA_DIMENSIONS, \
        #        1, \
        #        (self.imageWidth, self.imageHeight))
        self.inputImage = inputImage
        self.hsvMin = thresholdMin
        self.hsvMax = thresholdMax
        #self.inputImage = cv2.undistort(inputImage, self.CAMERA_MATRIX, self.CAMERA_DISTORTION, None) #newCameraMatrix)

        # Convert to HSV
        hsvImage = cv2.cvtColor(self.inputImage, cv2.COLOR_BGR2HSV)

        # Get masks
        self.mask = self.getMax(hsvImage)

        # self.orangeMask = self.getOrangeMask(hsvImage)
        # self.greenMask = self.getGreenMask(hsvImage)
        # self.totalMask = cv2.bitwise_or(self.orangeMask, self.greenMask)

        # # Get Contours
        # self.contours = self.getContours(self.mask)
        # # filter contours to select best
        # self.bestContour = None
        # self.bestHeight = None
        # self.bestWidth = None
        # self.bestxTop = None
        # self.bestyTop = None
        # self.filterContours()

    def getMask(self, hsvImage):
        # Threshold
        mask = cv2.inRange(hsvImage, self.hsvMin, self.hsvMax)
        # Erode
        mask = cv2.erode(mask, None, iterations = 1)
        # Dilate
        mask = cv2.dilate(mask, None, iterations = 3)
        return mask

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

    # def templateMatching(self):
    #     self.REFERENCE = cv2.imread('/home/racecar/racecar-ws/src/lab4/src/ConeTracking/cone.png')
    #     matchValues = []
    #     for contour in self.contours:
    #         xTop, yTop, width, height = cv2.boundingRect(contour)

    #         refHeight, refWidth =  self.REFERENCE.shape[:2]

    #         # resize the reference image to be the size of the bounding box
    #         # of the contour
    #         referenceScaled = cv2.resize(\
    #                 self.REFERENCE, \
    #                 (width, height))

    #         referenceHSV = cv2.cvtColor(referenceScaled, cv2.COLOR_BGR2HSV)
    #         referenceMask = self.getMask(referenceHSV)
    #         referenceContours = self.getContours(referenceMask)
    #         if referenceContours:
    #             referenceContour = referenceContours[0]
    #         else:
    #             matchValues.append(float('inf')) # good match values are closer to zero
    #             continue

    #         matchValue = cv2.matchShapes(referenceContour, contour, 1, 0)
    #         matchValues.append(matchValue)

    #     return matchValues

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
        # matchValues = self.templateMatching() - works well, but not needed for our purposes
        self.filteredContours = []
        bestArea = 0
        self.bestWidth = None

        for i in range(len(self.contours)):
            contour = self.contours[i]
            # matchValue = matchValues[i]
            xTop, yTop, width, height = cv2.boundingRect(contour)
            aspectRatio = width/float(height)

            # if (matchValue < self.MATCH_MAX) and (self.ASPECT_MIN < aspectRatio < self.ASPECT_MAX):
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

    def getDistance(self):
        if self.bestWidth:
            return self.DIST_TIMES_FOCAL_LENGTH_PX/self.getHeight()
        else:
            return -1

    def getAngle(self):
        if self.bestWidth:
            return (self.bestxTop + 0.5*self.bestWidth)/self.imageWidth - .5
        else:
            return -1

    def getBoundedImage(self):
        boundedImage = self.getMaskedImage()
        self.filterContours()
        # matchValues = self.templateMatching()

        for i in range(len(self.contours)):
            contour = self.contours[i]
            xTop, yTop, width, height = cv2.boundingRect(contour)
            # matchValue = matchValues[i]
            aspectRatio = width/float(height)
            area = width * height

            if contour is self.bestContour:
                color = [0,255,0]
            elif np.array(contour in self.filteredContours).any():
                color = [0,255,255]
            else:
                color = [0,0,255]

            # text = "TM: " + str(matchValue)[:5] + "\n" + "AR: " + str(aspectRatio)[:5]
            text = "AR: " + str(aspectRatio)[:4]
            text += "D: " + str(self.DIST_TIMES_FOCAL_LENGTH_PX/height)[:4] +\
                    "A: " + str(self.getAngle())[:4]

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
                    2,\
                    color,\
                    2)

        return boundedImage


# Testing code
# img = cv2.imread("/home/katy/racecar-ws/src/final_challenge/FinalChallengePy/Vision/images/redgreen10.png")

# c = ConeThreshold(img)

# cv2.imshow("original", img)

# cv2.imshow("masked", c.getMaskedImage())
# cv2.waitKey(0)

# cv2.destroyAllWindows()