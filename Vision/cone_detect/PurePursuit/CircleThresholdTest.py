from CircleThreshold import CircleThreshold
import cv2
import numpy as np

ct = CircleThreshold(150)

image = cv2.imread('../../circle_pics/angle1.png')
x, y = ct.findGoalPoint(image)
print(image.shape[-2])
print (x, y)
print "distance = ", np.linalg.norm([x, y])
pixelX, pixelY = ct.CoordinateTransformations.transformWorldToPixels((x,y))
print(pixelX, pixelY)
img = image.copy()
cv2.circle(img, (pixelX, pixelY), 3, [0,0,255], thickness = 2)


cv2.imwrite('angle1_threshold.png', img)