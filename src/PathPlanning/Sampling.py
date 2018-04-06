import numpy as np

from FinalChallengePy.Utils.RobotState import RobotState
from FinalChallengePy.PathPlanning.MiniSteer import MiniSteer

class Sampling:
    @staticmethod
    def getGaussianState(state, positionStdDev, angleStdDev, backwards=False):
        x, y = np.random.normal(loc=state.getPosition(), scale=positionStdDev)
        theta = np.random.normal(loc=state.getTheta(), scale=angleStdDev)
        
        if backwards:
            backwards = bool(np.random.randint(2))
        
        return RobotState(x, y, theta, backwards)
    
    @staticmethod
    def getUniformState(unoccupiedPoints, backwards=False):
        
        index = np.random.randint(len(unoccupiedPoints))
        position = unoccupiedPoints[index]
        
        theta = np.random.uniform(0, 2*np.pi)
        
        if backwards:
            backwards = bool(np.random.randint(2))
            
        return RobotState(position[0], position[1], theta, backwards)

    @staticmethod
    def getMiniState(state, angleStdDev, lengthStdDev, backwards=False):
        angle0 = abs(np.random.normal(scale=angleStdDev))
        angle1 = abs(np.random.normal(scale=angleStdDev))
        length = abs(np.random.normal(scale=lengthStdDev))

        side = 2*np.random.randint(2) - 1

        if backwards:
            backwards = bool(np.random.randint(2))

        return MiniSteer.getMiniSteerState(
            state, 
            angle0, 
            angle1, 
            length,
            side,
            backwards)
