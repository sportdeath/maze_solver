import numpy as np

def getTransformation(sources, destinations):
    numPoints = len(sources)
    A = np.zeros((2*numPoints, 8))
    b = np.zeros(8)

    for i in xrange(numPoints):
        # The x -> x' pairs
        A[i,0] = sources[i][0]
        A[i,1] = sources[i][1]
        A[i,2] = 1.
        A[i,6] = -sources[i][0]*destinations[i][0]
        A[i,7] = -sources[i][1]*destinations[i][0]

        b[i] = destinations[i][0]

        # The y -> y' pairs
        A[numPoints + i,3] = sources[i][0]
        A[numPoints + i,4] = sources[i][1]
        A[numPoints + i,5] = 1.
        A[numPoints + i,6] = -sources[i][0]*destinations[i][1]
        A[numPoints + i,7] = -sources[i][1]*destinations[i][1]

        b[numPoints + i] = destinations[i][1]

    Hv = np.dot(np.linalg.inv(A), b)

    H = np.array(
            [[Hv[0], Hv[1], Hv[2]],
             [Hv[3], Hv[4], Hv[5]],
             [Hv[6], Hv[7],   1. ]])

    return H
