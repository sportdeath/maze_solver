import numpy as np

from FinalChallengePy.PathPlanning.RobotState import RobotState

class Sampling:
    @staticmethod
    def getRandomState(state, positionStdDev, angleStdDev, backwards=False):
        x, y = np.random.normal(loc=state.getPosition, scale=self.positionStdDev)
        theta = np.random.normal(loc=state.getTheta(), scale=self.angleStdDev))
        
        if backwards:
            backwards = bool(np.random.randomint(2))
        
        return RobotState(x, y, theta, backwards)
