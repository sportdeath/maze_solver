import numpy as np

from FinalChallengePy.Utils.RobotState import RobotState
from FinalChallengePy.Utils.LocalGlobalUtils import LocalGlobalUtils
from FinalChallengePy.PathPlanning.Steer import MIN_RADIUS

class MiniSteer:
    @staticmethod
    def getMiniSteerState(state, angle0, angle1, length, side, backwards):
        arc0State = MiniSteer.stateAfterArc(state, angle0, side, backwards)

        lineState = MiniSteer.stateAfterLine(arc0State, length, backwards)

        arc1State = MiniSteer.stateAfterArc(lineState, angle1, side, backwards)

        return arc1State

    @staticmethod
    def stateAfterLine(state, length, backwards):
        if backwards:
            length = -length

        newPosition = state.getPosition() + state.getOrientation() * length

        return RobotState(
            newPosition[0],
            newPosition[1],
            state.getTheta(),
            backwards)

    @staticmethod
    def stateAfterArc(state, angle, side, backwards):
        if backwards:
            angle = -angle

        x = side * (1 - np.cos(angle))
        y = np.sin(angle)

        pointLocal = MIN_RADIUS * np.array([x, y])
        pointGlobal = LocalGlobalUtils.localToGlobal(state, pointLocal)

        return RobotState(
            pointGlobal[0], 
            pointGlobal[1], 
            state.getTheta() - side * angle,
            backwards)
