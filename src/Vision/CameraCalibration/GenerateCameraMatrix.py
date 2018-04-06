#!/usr/bin/env python

'''
camera calibration for distorted images with chess board samples
reads distorted images, calculates the calibration and write undistorted images

usage:
    calibrate.py [--debug <output path>] [--square_size] [<image mask>]

default values:
    --debug:    ./output/
    --square_size: 1.0
    <image mask> defaults to ../data/left*.jpg
'''

from __future__ import print_function

import numpy as np
import cv2
import os
import sys
import getopt
from glob import glob
from common import splitfn

from FinalChallengePy.Vision.Constants import *
from FinalChallengePy.Vision.CoordinateTransformations import CoordinateTransformations

if __name__ == '__main__':

    args, img_mask = getopt.getopt(sys.argv[1:], '', ['debug=', 'square_size='])
    args = dict(args)
    args.setdefault('--debug', './output/')
    args.setdefault('--square_size', .001*7.2*0.99623793403)
    if not img_mask:
        img_mask = './input/*.png'  # default
    else:
        img_mask = img_mask[0]

    img_names = glob(img_mask)
    debug_dir = args.get('--debug')
    if not os.path.isdir(debug_dir):
        os.mkdir(debug_dir)
    square_size = float(args.get('--square_size'))

    pattern_size = (9, 6)
    pattern_points = np.zeros((np.prod(pattern_size), 3), np.float32)
    pattern_points[:, :2] = np.indices(pattern_size).T.reshape(-1, 2)
    pattern_points *= square_size

    obj_points = []
    img_points = []
    imgs_found = []
    h, w = 0, 0
    img_names_undistort = []
    for fn in img_names:
        print('processing %s... ' % fn, end='')
        img = cv2.imread(fn, 0)
        if img is None:
            print("Failed to load", fn)
            continue

        h, w = img.shape[:2]
        found, corners = cv2.findChessboardCorners(img, pattern_size, flags=1)
        if found:
            # Get corners in pixel coordinates
            term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.1)
            cv2.cornerSubPix(img, corners, (5, 5), (-1, -1), term)

            # Visualize 
            vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            cv2.drawChessboardCorners(vis, pattern_size, corners, found)
            path, name, ext = splitfn(fn)
            outfile = debug_dir + name + '_Highlighted.png'
            cv2.imwrite(outfile, vis)

            imgs_found.append(fn)
            img_points.append(corners.reshape(-1, 2))
            obj_points.append(pattern_points)
        else:
            print('chessboard not found')
            continue

        print('ok')

    # calculate camera distortion
    rms, camera_matrix, dist_coefs, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, (w, h), None, None)
    pixelsPerSquare = np.matmul(camera_matrix, np.array([square_size, 0, 0]))[0]

    print("\nRMS:", rms)
    print("camera matrix:\n", camera_matrix)
    print("distortion coefficients: ", dist_coefs.ravel())
    print("width, height = ", (w, h))
    print("pixelsPerSquare: ", pixelsPerSquare)

    np.savetxt(CAMERA_MATRIX_FILE, camera_matrix)
    np.savetxt(CAMERA_MATRIX_INV_FILE, np.linalg.inv(camera_matrix))
    print("Camera matrix written")

    cv2.destroyAllWindows()
