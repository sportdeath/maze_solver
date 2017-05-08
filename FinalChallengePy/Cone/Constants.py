import numpy as np

HSV_MIN_ORANGE = np.array([2, 150, 133])
HSV_MAX_ORANGE = np.array([10, 254, 255])

HSV_MIN_GREEN = np.array([74, 30, 30])
HSV_MAX_GREEN = np.array([109, 133, 85])

RED_CONE_DIRECTION = 1
GREEN_CONE_DIRECTION = -1

DISTANCE_THRESHOLD = 10 * 0.3048/1 # 10 ft in meters