#!/usr/bin/env python

import cv2

import numpy as np

INPUT = "basement_fixed.png"
OUTPUT = "basement_fixed_dilated.png"

CAR_WIDTH = .3 # Meters
RESOLUTION = 0.05 # Meters per Pixels
ERROR = 0.15 # Meters
RADIUS = int((CAR_WIDTH + ERROR)/RESOLUTION)

THRESH_MIN = 254
THRESH_MAX = 255

if __name__ == "__main__":
    img = cv2.imread(
            INPUT,
            cv2.IMREAD_GRAYSCALE)

    img = cv2.inRange(
            img, 
            THRESH_MIN,
            THRESH_MAX)

    cv2.imwrite(OUTPUT+"Preprocessing.jpg",img)

    img = 255 - img


    kernel = np.zeros((2*RADIUS,2*RADIUS,1), np.uint8)
    cv2.circle(
            kernel, 
            (RADIUS, RADIUS), 
            RADIUS, 
            THRESH_MAX,
            -1)

    img = cv2.filter2D(img, -1, kernel)

    img = 255 - img

    img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)

    cv2.imwrite(OUTPUT,img)
    cv2.imwrite(OUTPUT+"kernel.png",kernel)

