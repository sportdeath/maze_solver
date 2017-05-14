import numpy as np

from rtree import index

from FinalChallengePy.PathPlanning.Steer import Steer
from FinalChallengePy.PathPlanning.Sampling import Sampling
from FinalChallengePy.PathPlanning.Constants import *

from FinalChallengePy.Utils.RobotState import RobotState
from FinalChallengePy.Utils.MapUtils import MapUtils

class RRTNode:
    def __init__(self, state, cost, parent):
        self.state = state # RobotState
        self.cost = cost
        self.parent = parent

class RRT:
    def __init__(self, 
            mapMsg, 
            rangeMethod=None, 
            maxIterations = DEFAULT_MAX_RRT_ITERATIONS, 
            numOptimizations = DEFAULT_NUM_OPTIMIZATIONS, 
            verbose = False,
            gaussianProbability = 0.,
            gaussianPositionStdDev = 0.5,
            gaussianAngleStdDev = 0.5,
            miniSteerProbability = 0,
            miniSteerAngleStdDev = 0.1,
            miniSteerLengthStdDev = 0.1
            ):
        self.mapMsg = mapMsg
        if rangeMethod is None:
            self.rangeLib = MapUtils.getRangeLib(self.mapMsg)
            self.rangeMethod = MapUtils.getRangeMethod(self.rangeLib, self.mapMsg)
        else:
            self.rangeMethod = rangeMethod
        self.unoccupiedPoints = MapUtils.getUnoccupiedPoints(self.mapMsg)
        self.maxIterations = maxIterations
        self.numOptimizations = numOptimizations
        self.verbose = verbose
        self.gaussianProbability = gaussianProbability
        self.gaussianPositionStdDev = gaussianPositionStdDev
        self.gaussianAngleStdDev = gaussianAngleStdDev
        self.miniSteerProbability = miniSteerProbability
        self.miniSteerAngleStdDev = miniSteerAngleStdDev
        self.miniSteerLengthStdDev = miniSteerLengthStdDev

    @staticmethod
    def nnTreeInsertLastElement(nnTree, tree):
        index = len(tree) - 1
        obj = tree[index]
        x = obj.state.getPosition()[0]
        y = obj.state.getPosition()[1]
        nnTree.insert(index, (x, y, x, y))

    @staticmethod
    def getKNearest(state, nnTree, tree, k):
        x = state.getPosition()[0]
        y = state.getPosition()[1]
        return [tree[i] for i in nnTree.nearest((x, y, x, y), k)]

    def computePath(self, init, goal, backwards=False, fakeWalls = None, multipleGoals = False, sampleStates = []):
        tree = [RRTNode(init, 0, None)]
        nnTree = index.Index()
        RRT.nnTreeInsertLastElement(nnTree, tree)

        if multipleGoals:
            goalStates = goal
        else:
            goalStates = [goal]

        if backwards:
            newGoalStates = []
            for g in goalStates:
                backwardsGoal = RobotState(
                        g.getX(),
                        g.getY(),
                        g.getTheta(),
                        not g.isBackwards())
                newGoalStates.append(backwardsGoal)
            goalStates += newGoalStates

        # Building paths to goal
        newState = True
        bestCost = np.inf
        bestGoal = None
        bestGoalIndex = -1

        newState = True
        i = 0

        while self.maxIterations < 0 or i < self.maxIterations:
            if self.verbose:
                if i % 100 == 0:
                    print i, "/", self.maxIterations
            newState, goal, goalIndex = self.updateTree(
                    tree,
                    nnTree,
                    newState,
                    goalStates,
                    backwards, 
                    fakeWalls = fakeWalls, 
                    sampleStates = sampleStates)
            if goal:
                if tree[-1].cost < bestCost:
                    bestGoalIndex = goalIndex
                    bestGoal = tree[-1]
                    bestCost = tree[-1].cost

                if self.maxIterations < 0:
                    break
            i += 1

        if bestGoal:
            if self.verbose:
                print("Found goal", bestGoalIndex)
            self.path = RRT.treeToPath(tree, bestGoal)
            # self.optimize(sampleStates, backwards, fakeWalls)
            return bestGoalIndex, tree

        self.path = [(init.getPosition(), False)]
        return -1, tree

    def updateTree(self, tree, nnTree, newState, goalStates, backwards, fakeWalls = None,  sampleStates = []):
        # Check whether last state we added 
        # has steerable path to goal
        if newState:
            for goalIndex, goal in enumerate(goalStates):
                steer = Steer(tree[-1].state, goal, self.rangeMethod)

                if steer.isSteerable(fakeWalls = fakeWalls):
                    # If is is add the goal state to
                    # the tree and return
                    tree.append(
                            RRTNode(
                                goal, 
                                tree[-1].cost + steer.getLength(),
                                tree[-1]))
                    return False, True, goalIndex
            newState = False

        randomState = self.getRandomState(sampleStates, tree, nnTree, backwards)

        minNode = None
        minCost = np.inf

        for node in RRT.getKNearest(randomState, nnTree, tree, K):
            steer = Steer(node.state, randomState, self.rangeMethod)
            if steer.isSteerable(fakeWalls = fakeWalls):
                length = steer.getLength()
                if length + node.cost < minCost:
                    minCost = length + node.cost
                    minNode = node

        # If there is no steerable path, minNode will be None
        if minNode:
            newNode = RRTNode(randomState, minCost, minNode)
            tree.append(newNode)
            RRT.nnTreeInsertLastElement(nnTree, tree)
            newState = True

        return newState, False, -1

    def getRandomState(self, sampleStates, tree, nnTree, backwards):

        random = np.random.rand()

        if random < self.gaussianProbability and len(sampleStates) > 0:
            choice = 1
        elif random < self.gaussianProbability + self.miniSteerProbability:
            choice = 2
        else:
            choice = 0

        if choice == 2:
            # Generate a random point
            randomState = Sampling.getUniformState(self.unoccupiedPoints)

            # find k nearest neighbors
            nns = RRT.getKNearest(randomState, nnTree, tree, K)

            # choose one randomly
            index = np.random.randint(len(nns))
            nnState = nns[index].state

            # generate mini state from that point
            return Sampling.getMiniState(
                    nnState, 
                    self.miniSteerAngleStdDev, 
                    self.miniSteerLengthStdDev, 
                    backwards)

        elif choice == 1:
            # Choose state
            index = np.random.randint(len(sampleStates))
            state = sampleStates[index]

            # draw from Gaussian around it
            return Sampling.getGaussianState(state, self.gaussianPositionStdDev, self.gaussianAngleStdDev)

        else:
            return Sampling.getUniformState(self.unoccupiedPoints, backwards)

    @staticmethod
    def treeToPath(tree, bestGoal):
        if bestGoal:
            path = [bestGoal]

            while path[-1].parent:
                path.append(path[-1].parent)

            path.reverse()

            return path

    def treeToLineList(self, tree):
        lineList = []
        for node in tree:
            if node.parent != None:
                steer = Steer(node.parent.state, node.state, self.rangeMethod)
                points = steer.getPoints()
                for i in xrange(len(points)):
                    lineList.append(points[i])
                    if i != 0 and i != len(points) - 1:
                        lineList.append(points[i])
        return lineList

    def getLineLists(self):
        forwardList = []
        backwardList = []

        paths = self.getPoints(self.path, oriented = True)

        for path in paths:
            if path[1]:
                l = backwardList
            else:
                l = forwardList
            points = path[0]
            for i in xrange(len(points)):
                l.append(points[i])
                if i != 0 and i != len(points) - 1:
                    l.append(points[i])

        return (forwardList, backwardList)

    def reverse(self):
        for i in xrange(len(self.path) - 1):
            self.path[i].state = RobotState(
                    self.path[i].state.getPosition()[0],
                    self.path[i].state.getPosition()[1],
                    self.path[i].state.getTheta(),
                    not self.path[i+1].state.isBackwards())

        self.path.reverse()

    def getPath(self):
        return self.path

    def getSteers(self, path=None):
        if path is None:
            path = self.path
        steers = []
        previousNode = path[0]
        for i in xrange(1,len(path)):
            node = path[i]
            steer = Steer(previousNode.state, node.state, self.rangeMethod)
            steers.append(steer)
            previousNode = node
        return steers

    def getPoints(self, path, oriented = False, goalExtension=0.):
        paths = []
        previousNode = path[0]

        for i in xrange(1,len(path)):
            node = path[i]
            steer = Steer(previousNode.state, node.state, self.rangeMethod)

            # only add extension if it is the last of its sign...
            if i == len(path) - 1 or node.state.isBackwards() != path[i+1].state.isBackwards():
                extension = goalExtension
            else:
                extension = 0.

            # if it is the same,
            # combine it with the previous path
            if i != 1 and node.state.isBackwards() == path[i-1].state.isBackwards():
                points = steer.getPoints(goalExtension=extension)
                if oriented:
                    paths[-1] = (paths[-1][0] + points, paths[-1][1])
                else:
                    paths[-1] += points
            else:
                paths.append(steer.getPoints(oriented=oriented, goalExtension=extension))

            previousNode = node

        return paths

    def optimize(self, states, backwards, fakeWalls = None):
        if len(self.path) < 3:
            return

        for i in xrange(self.numOptimizations):
            # Ignore the end points
            randomState = self.getRandomState(states, backwards)

            for i in xrange(1,len(self.path)-1):
                # Steer to that path
                steer1 = Steer(self.path[i - 1].state, randomState, self.rangeMethod)
                steer2 = Steer(randomState, self.path[i + 1].state, self.rangeMethod)

                if steer1.isSteerable(fakeWalls = fakeWalls) and steer2.isSteerable(fakeWalls = fakeWalls):
                    oldCost = (self.path[i].cost - self.path[i - 1].cost) \
                            + (self.path[i + 1].cost - self.path[i].cost)
                    newCost = steer1.getLength() + steer2.getLength()
                    if newCost < oldCost:
                        self.path[i] = RRTNode(
                                randomState, 
                                self.path[i - 1].cost + steer1.getLength(),
                                self.path[i- 1])
