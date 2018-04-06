import numpy as np
from FinalChallengePy.Utils.GeomUtils import GeomUtils

class LocalGlobalUtils:
    @staticmethod
    def globalToLocal(state, globalPoint):
        translatedPoint = globalPoint - state.getPosition()

        # rotate into the up frame of reference
        angle = np.arccos(state.getOrientation()[1]) \
                * np.sign(state.getOrientation()[0])

        return GeomUtils.rotateVector(translatedPoint, angle)

    @staticmethod
    def localToGlobal(state, localPoint):
        angle = - np.arccos(state.getOrientation()[1]) \
                * np.sign(state.getOrientation()[0])

        translatedPoint = GeomUtils.rotateVector(localPoint, angle)

        return translatedPoint + state.getPosition()
