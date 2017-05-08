import numpy as np

def getTransformation(sources, destinations):
    sourceDiff = sources[1] - sources[0]
    destinationDiff = destinations[1] - destinations[0]

    sourceNorm = np.linalg.norm(sourceDiff)
    destinationNorm = np.linalg.norm(destinationDiff)

    # Determine the scaling factor
    scalingFactor = sourceNorm/destinationNorm

    # Determine the rotation matrix
    sourceUnit = sourceDiff/sourceNorm
    destinationUnit = destinationDiff/destinationNorm

    axis = np.cross(sourceUnit, destinationUnit)
    cos = np.dot(sourceUnit, destinationUnit)
    crossProductMatrix = np.array(
            [[0, -axis[2], axis[1]],
             [axis[2], 0, -axis[0]],
             [-axis[1], axis[0], 0]])

    rotationMatrix = np.identity(3) + \
            crossProductMatrix + \
            (np.dot(crossProductMatrix, crossProductMatrix) * (1/(1+cos)))

    # Determine the translation vector
    translationVector = scalingFactor*destinations[0] - np.dot(rotationMatrix, sources[0])

    return (scalingFactor, rotationMatrix, translationVector)

if __name__ == "__main__":
    # Test with random vectors
    randomVectors = []
    for i in xrange(4):
        randomVector = np.random.rand(3)
        randomVectors.append(randomVector)

    sources = [randomVectors[0], randomVectors[1]]
    destinations = [randomVectors[2], randomVectors[3]]

    (scalingFactor, rotationMatrix, translationVector) = \
            getTransformation(sources, destinations)

    for i in xrange(2):
        output = (np.dot(rotationMatrix, sources[i]) + translationVector)/scalingFactor
        print("This should be zero: ", destinations[i] - output)
