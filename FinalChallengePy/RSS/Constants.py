from FinalChallengePy.CarConstants import *

import os
CONE_PATH = os.path.split(__file__)[0]

CONE_WIDTH = 0.3
BIG_PATH_FILE = os.path.join(CONE_PATH, "BigPath.pickle")
RSS_OFFSET = CONE_WIDTH + CAR_WIDTH/2. + 0.1
RSS_NUM_POINTS = 6

RSS_MAX_ITERATIONS = 20
RSS_NUM_OPTIMIZATIONS = 0

CONE_DISTANCE_THRESHOLD = (3.048 - 1.)/2.
