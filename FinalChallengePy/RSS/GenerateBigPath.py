#!/usr/bin/env python

import numpy as np

import rospy

from FinalChallengePy.PathPlanning.MapUtils import MapUtils
from FinalChallengePy.PathPlanning.RobotState import RobotState
from FinalChallengePy.PathPlanning.RRT import RRT

from FinalChallengePy.RSS.Constants import *
from FinalChallengePy.TrajectoryTracking.Constants import *

if __name__=="__main__":
    rospy.init_node("GenerateBigPath")

    mapMsg = MapUtils.getMap()
    RRT = RRT(mapMsg)

    initState = RobotState(-1,0,3.14)
    goalState = RobotState(2.15,2.38,-1.57)

    RRT.computePath(initState, goalState)
    np.savetxt(BIG_PATH_FILE, RRT.getPaths(goalExtension = LOOK_AHEAD_DISTANCE)[0])
