import os
VISION_PATH = os.path.split(__file__)[0]
CONSTANTS_PATH = os.path.join(VISION_PATH, "Constants")

CAMERA_MATRIX_FILE = os.path.join(CONSTANTS_PATH, "CameraMatrix.txt")
CAMERA_MATRIX_INV_FILE = os.path.join(CONSTANTS_PATH, "CameraMatrixInv.txt")
ROTATION_MATRIX_FILE = os.path.join(CONSTANTS_PATH, "RotationMatrix.txt")
ROTATION_MATRIX_INV_FILE = os.path.join(CONSTANTS_PATH, "RotationMatrixInv.txt")
TRANSLATION_VECTOR_FILE = os.path.join(CONSTANTS_PATH, "TranslationVector.txt")
SCALING_FACTOR_FILE = os.path.join(CONSTANTS_PATH, "ScalingFactor.txt")
