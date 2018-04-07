import os
VISION_PATH = os.path.split(__file__)[0]
CONSTANTS_PATH = os.path.join(VISION_PATH, "Constants")
IMAGES_PATH = os.path.join(VISION_PATH, "images")

TEST_IMAGE = os.path.join(IMAGES_PATH, "stata1.png")
TEST_IMAGE_OUT = os.path.join(VISION_PATH, "test.png")

CAMERA_MATRIX_FILE = os.path.join(CONSTANTS_PATH, "CameraMatrix.txt")
CAMERA_MATRIX_INV_FILE = os.path.join(CONSTANTS_PATH, "CameraMatrixInv.txt")
TRANSFORMATION_MATRIX_FILE = os.path.join(CONSTANTS_PATH, "TransformationMatrix.txt")
TRANSFORMATION_MATRIX_INV_FILE = os.path.join(CONSTANTS_PATH, "TransformationMatrixInv.txt")
