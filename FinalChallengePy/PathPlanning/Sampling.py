import numpy as np

from FinalChallengePy.PathPlanning.RobotState import RobotState

class Sampling:
    def __init__(self, currentState, positionSampleWidth, angleSampleWidth):
        self.currentState = currentState
        self.currentPosition = currentState.getPosition()

        self.positionSampleWidth = positionSampleWidth
        self.angleSampleWidth = angleSampleWidth

    def getRandomState(self):
        newX, newY = np.random.normal(loc=self.currentPosition, scale=self.positionSampleWidth)

        newTheta = np.random.normal(loc=self.currentState.getTheta(), scale=self.angleSampleWidth)
        newTheta = self.recenterTheta(newTheta)

        randomState = RobotState(newX, newY, newTheta, self.currentState.isBackwards())

        return randomState

    def recenterTheta(self, theta):
        while theta < 0:
            theta += 2*np.pi
        while theta > 2*np.pi:
            theta -= 2*pi

        return theta