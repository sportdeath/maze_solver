#!/usr/bin/env python

import numpy as np

import rospy

from FinalChallengePy.Utils.MapUtils import MapUtils
from FinalChallengePy.Utils.RobotState import RobotState

from FinalChallengePy.PathPlanning.RRT import RRT

from FinalChallengePy.RSS.Constants import *
from FinalChallengePy.TrajectoryTracking.Constants import *

import pickle

if __name__=="__main__":
    rospy.init_node("GenerateBigPath")

    mapMsg = MapUtils.getMap()
    RRT = RRT(mapMsg, maxIterations=5000, numOptimizations=1000, verbose=True)

    initState = RobotState(-1,0,3.14)
    goalState = RobotState(2.15,2.38,-1.57)

    RRT.computePath(initState, goalState)
    steers = RRT.getSteers()
    print(steers)
    pickle.dump(steers, open(BIG_PATH_FILE, 'wb'))
