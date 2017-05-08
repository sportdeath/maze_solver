import numpy as np

from FinalChallengePy.PathPlanning.Steer import Steer
from FinalChallengePy.PathPlanning.RobotState import RobotState
from FinalChallengePy.PathPlanning.MapUtils import MapUtils

class RRTNode:
    def __init__(self, state, cost, parent):
        self.state = state # RobotState
        self.cost = cost
        self.parent = parent

class RRT:
    MAX_RRT_ITERATIONS = 100
    MAX_OPTIMIZATION_ITERATIONS = 100

    def __init__(self, mapMsg):
        self.mapMsg = mapMsg
        self.rangeLib = MapUtils.getRangeLib(self.mapMsg)
        self.rangeMethod = MapUtils.getRangeMethod(self.rangeLib, self.mapMsg)
        self.unoccupiedPoints = MapUtils.getUnoccupiedPoints(self.mapMsg)

    def computePath(self, init, goal, backwards=False):
        tree = [RRTNode(init, 0, None)]

        goalStates = [goal]
        if backwards:
            backwardsGoal = RobotState(
                    goal.getX(),
                    goal.getY(),
                    goal.getTheta(),
                    not goal.isBackwards())
            goalStates.append(backwardsGoal)

        # Building paths to goal
        newState = True
        bestCost = np.inf
        bestGoal = None

        newState = True
        RRT_ITERATIONS = 0
        while RRT_ITERATIONS < self.MAX_RRT_ITERATIONS:
            newState = self.updateTree(tree,newState,goalStates,backwards)
            if newState == "Goal":
                if tree[-1].cost < bestCost:
                    bestGoal = tree[-1]
                    bestCost = tree[-1].cost

            RRT_ITERATIONS += 1

        if bestGoal:
            self.path = RRT.treeToPath(tree, bestGoal)
            self.optimize(backwards)
            return tree

        print "NO PATH FOUND"
        return [(init.getPosition(), False)]

    def updateTree(self, tree, newState, goalStates, backwards):
        # Check whether last state we added 
        # has steerable path to goal
        if newState:
            for goal in goalStates:
                steer = Steer(tree[-1].state, goal, self.rangeMethod)

                if steer.isSteerable():
                    # If is is add the goal state to
                    # the tree and return
                    tree.append(
                            RRTNode(
                                goal, 
                                tree[-1].cost + steer.getLength(),
                                tree[-1]))
                    return "Goal"
            newState = False

        randomState = self.getRandomState(backwards)

        minNode = None
        minCost = np.inf

        for node in tree:
            steer = Steer(node.state, randomState, self.rangeMethod)
            if steer.isSteerable():
                length = steer.getLength()
                if length + node.cost < minCost:
                    minCost = length + node.cost
                    minNode = node

        # If there is no steerable path, minNode will be None
        if minNode:
            newNode = RRTNode(randomState, minCost, minNode)
            tree.append(newNode)
            newState = True

        return newState

    def getRandomState(self, backwards):
        # TODO
        # Right now this is uniform
        # How can we change how we sample?
        randomIndex = np.random.randint(len(self.unoccupiedPoints))
        randomPosition = self.unoccupiedPoints[randomIndex]
        randomTheta = np.random.uniform(0, 2*np.pi)
        if backwards:
            backwards = bool(np.random.randint(2))
        else:
            backwards = False
        randomState = RobotState(randomPosition[0], randomPosition[1], randomTheta, backwards)
        return randomState

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

    def getPaths(self, oriented = False, goalExtension=0.):
        paths = []
        previousNode = self.path[0]

        for i in xrange(1,len(self.path)):
            node = self.path[i]
            steer = Steer(previousNode.state, node.state, self.rangeMethod)

            # only add extension if it is the last of its sign...
            if i == len(self.path) - 1 or node.state.isBackwards() != self.path[i+1].state.isBackwards():
                extension = goalExtension
            else:
                extension = 0.

            # if it is the same,
            # combine it with the previous path
            if i != 1 and node.state.isBackwards() == self.path[i-1].state.isBackwards():
                points = steer.getPoints(goalExtension=extension)
                if oriented:
                    paths[-1] = (paths[-1][0] + points, paths[-1][1])
                else:
                    paths[-1] += points
            else:
                paths.append(steer.getPoints(oriented=oriented, goalExtension=extension))

            previousNode = node

        return paths

    def optimize(self, backwards):
        if len(self.path) < 3:
            return

        for i in xrange(self.MAX_OPTIMIZATION_ITERATIONS):
            # Ignore the end points
            randomState = self.getRandomState(backwards)

            for index in xrange(1,len(self.path)-1):
                # Steer to that path
                steer1 = Steer(self.path[index - 1].state, randomState, self.rangeMethod)
                steer2 = Steer(randomState, self.path[index + 1].state, self.rangeMethod)

                if steer1.isSteerable() and steer2.isSteerable():
                    oldCost = (self.path[index].cost - self.path[index - 1].cost) \
                            + (self.path[index + 1].cost - self.path[index].cost)
                    newCost = steer1.getLength() + steer2.getLength()
                    if newCost < oldCost:
                        self.path[index] = RRTNode(
                                randomState, 
                                self.path[index - 1].cost + steer1.getLength(),
                                self.path[index - 1])
