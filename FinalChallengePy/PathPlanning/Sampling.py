import numpy as np

from FinalChallengePy.Utils.RobotState import RobotState

class Sampling:
    @staticmethod
    def getGaussianState(state, positionStdDev, angleStdDev, backwards=False):
        x, y = np.random.normal(loc=state.getPosition(), scale=positionStdDev)
        theta = np.random.normal(loc=state.getTheta(), scale=angleStdDev)
        
        if backwards:
            backwards = bool(np.random.randomint(2))
        
        return RobotState(x, y, theta, backwards)
    
    @staticmethod
    def getUniformState(unoccupiedPoints, backwards=False):
        
        index = np.random.randint(len(unoccupiedPoints))
        position = unoccupiedPoints[index]
        
        theta = np.random.uniform(0, 2*np.pi)
        
        if backwards:
            backwards = bool(np.random.randint(2))
            
        return RobotState(position[0], position[1], theta, backwards)
