import numpy as np

from rtree import index

from FinalChallengePy.PathPlanning.Steer import Steer
from FinalChallengePy.PathPlanning.RobotState import RobotState
from FinalChallengePy.PathPlanning.MapUtils import MapUtils
from FinalChallengePy.PathPlanning.Constants import *

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
            positionStdDev = 4.
            angleStdDev = 0.8
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
        self.positionStdDev = positionStdDev
        self.angleStdDev = angleStdDev

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

    def computePath(self, init, goal, backwards=False, fakeWall = None, multipleGoals = False):
        tree = [RRTNode(init, 0, None)]
        nnTree = index.Index()
        RRT.nnTreeInsertLastElement(nnTree, tree)

        if multipleGoals:
            goalStates = goal
        else:
            goalStates = [goal]

        if backwards:
            for g in goalStates:
                backwardsGoal = RobotState(
                        g.getX(),
                        g.getY(),
                        g.getTheta(),
                        not g.isBackwards())
                goalStates.append(backwardsGoal)

        # Building paths to goal
        newState = True
        bestCost = np.inf
        bestGoal = None
        bestGoalIndex = -1

        newState = True
        i = 0
        while self.maxIterations < 0 or i < self.maxIterations:
            if self.verbose:
                print(i)
            newState, goal, goalIndex = self.updateTree(tree,nnTree,newState,init,goalStates,backwards, fakeWall = fakeWall)
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
                print("Found goal, optimizing")
            self.path = RRT.treeToPath(tree, bestGoal)
            self.optimize([init]+goalStates, backwards, fakeWall)
            return bestGoalIndex, tree

        self.path = [(init.getPosition(), False)]
        return -1, tree

    def updateTree(self, tree, nnTree, newState, init, goalStates, backwards, fakeWall = None):
        # Check whether last state we added 
        # has steerable path to goal
        if newState:
            for goalIndex, goal in enumerate(goalStates):
                steer = Steer(tree[-1].state, goal, self.rangeMethod)

                if steer.isSteerable(fakeWall = fakeWall):
                    # If is is add the goal state to
                    # the tree and return
                    tree.append(
                            RRTNode(
                                goal, 
                                tree[-1].cost + steer.getLength(),
                                tree[-1]))
                    return False, True, goalIndex
            newState = False

        randomState = self.getRandomState([init]+goals, backwards)

        minNode = None
        minCost = np.inf

        for node in RRT.getKNearest(randomState, nnTree, tree, K):
            steer = Steer(node.state, randomState, self.rangeMethod)
            if steer.isSteerable(fakeWall = fakeWall):
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

    def getRandomState(self, states, backwards):
        choice = np.random.rand()
        if choice < self.gaussianProbability:
            # Choose state from either init or goal
            index = np.random.randint(len(states))
            state = states[index]
            return Sampling.getGaussianState(state, self.positionStdDev, self.angleStdDev)
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

        paths = self.getPaths(oriented = True)

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

    def optimize(self, states, backwards, fakeWall = None):
        if len(self.path) < 3:
            return

        for i in xrange(self.numOptimizations):
            # Ignore the end points
            randomState = self.getRandomState(states, backwards)

            for i in xrange(1,len(self.path)-1):
                # Steer to that path
                steer1 = Steer(self.path[i - 1].state, randomState, self.rangeMethod)
                steer2 = Steer(randomState, self.path[i + 1].state, self.rangeMethod)

                if steer1.isSteerable(fakeWall = fakeWall) and steer2.isSteerable(fakeWall = fakeWall):
                    oldCost = (self.path[i].cost - self.path[i - 1].cost) \
                            + (self.path[i + 1].cost - self.path[i].cost)
                    newCost = steer1.getLength() + steer2.getLength()
                    if newCost < oldCost:
                        self.path[i] = RRTNode(
                                randomState, 
                                self.path[i - 1].cost + steer1.getLength(),
                                self.path[i- 1])
