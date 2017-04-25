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

# Python 2/3 compatibility
from __future__ import print_function

import numpy as np
import cv2

# local modules
from common import splitfn
import sys
sys.path.insert(0, '../')
from CoordinateTransformations import CoordinateTransformations

# built-in modules
import os

if __name__ == '__main__':
    import sys
    import getopt
    from glob import glob

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

    # undistort the image with the calibration
    print('')
    for i in range(len(imgs_found)):
        img_found = imgs_found[i]
        img = cv2.imread(img_found)

        # undistort
        img = cv2.undistort(img, camera_matrix, dist_coefs, None)

        r = rvecs[i]
        r, jacobian = cv2.Rodrigues(r)

        t = tvecs[i]

        Hr = np.matmul(np.matmul(camera_matrix, np.linalg.inv(r)), np.linalg.inv(camera_matrix))

        C = np.matmul(np.linalg.inv(r), t)
        translation_vector = -np.matmul(camera_matrix,C)
        h,  w = img.shape[:2]
        # Calculate pixels per square
        translation_vector[0] = translation_vector[0] + (-8*pixelsPerSquare)/2.
        translation_vector[1] = translation_vector[1] + (-5*pixelsPerSquare)/2.
        translation_vector = translation_vector/C[2]
        translation_vector[2] = 1/C[2]

        Ht = np.identity(3)
        Ht[:,2:] = translation_vector

        H = np.matmul(Ht, Hr)
        print(H)

        transform = CoordinateTransformations(H, 0, 0)
        img2 = transform.displayCoordinatesOnWorld(img, 2.46, 30)

        img = cv2.warpPerspective(img,H,(w,h))
        cv2.circle(img,(w/2,h/2),2,[0,0,255],thickness=2)
        cv2.circle(img,(0,0),2,[0,0,255],thickness=2)

        path, name, ext = splitfn(img_found)
        outfile = debug_dir + name + '_undistorted.png'
        outfile2 = debug_dir + name + '_grid.png'
        print('Undistorted image written to: %s' % outfile)
        cv2.imwrite(outfile, img)
        cv2.imwrite(outfile2, img2)

    cv2.destroyAllWindows()
