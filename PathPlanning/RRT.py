import numpy as np

from Steer import Steer
from RobotState import RobotState
from MapUtils import MapUtils

class RRTNode:
    def __init__(self, state, cost, parent):
        self.state = state # RobotState
        self.cost = cost
        self.parent = parent

class RRT:
    MAX_ITERATIONS = 10000

    def __init__(self, mapMsg):
        self.mapMsg = mapMsg
        self.rangeLib = MapUtils.getRangeLib(self.mapMsg)
        self.rangeMethod = MapUtils.getRangeMethod(self.rangeLib, self.mapMsg)
        self.unoccupiedPoints = MapUtils.getUnoccupiedPoints(self.mapMsg)

    def computePath(self, init, goal):
        tree = [RRTNode(init, 0, None)]
        newState = True

        # Building paths to goal
        newState = True
        while True:
            print(len(tree))
            newState = self.updateTree(tree,newState,goal)
            if newState == "Done":
                print("Done!")
                self.path = RRT.treeToPath(tree)
                self.optimize()
                return tree

    def updateTree(self, tree, newState, goal):
        # Check whether last state we added 
        # has steerable path to goal
        if newState:
            steer = Steer(tree[-1].state, goal, self.rangeMethod)

            if steer.isSteerable():
                # If is is add the goal state to
                # the tree and return
                tree.append(
                        RRTNode(
                            goal, 
                            tree[-1].cost + steer.getLength(),
                            tree[-1]))
                return "Done"
            newState = False

        randomState = self.getRandomState()

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

    def getRandomState(self):
        # TODO
        # Right now this is uniform
        # How can we change how we sample?
        randomIndex = np.random.randint(len(self.unoccupiedPoints))
        randomPosition = self.unoccupiedPoints[randomIndex]
        randomTheta = np.random.uniform(0, 2*np.pi)
        randomState = RobotState(randomPosition[0], randomPosition[1], randomTheta)
        return randomState

    # returns a random RobotState centered about state.position and theta pointed generally in the right direction
    def getRandomStateCentered(self, state):
        STANDARD_DEVIATION = 1
        randomTheta = np.random.uniform(0, np.pi*2)
        randomXPosition = np.random.normal(state.position[0], STANDARD_DEVIATION)
        randomYPosition = np.random.normal(state.position[1], STANDARD_DEVIATION)

        return RobotState(randomXPosition, randomYPosition, randomTheta)

    @staticmethod
    def treeToPath(tree):
        # Goal node is always last node in tree
        path = [tree[-1]]

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

    def getPoints(self):
        points = []
        previousNode = self.path[0]

        for i in xrange(1,len(self.path)):
            node = self.path[i]
            steer = Steer(previousNode.state, node.state, self.rangeMethod)
            points += steer.getPoints()
            previousNode = node

        return points

    # TODO spend some time greedily optimizing path
    def optimize(self):
        print "Optimizing"
        for i in xrange(self.MAX_ITERATIONS):
            randomIndex = np.random.randint(len(self.path))
            randomState = self.getRandomStateCentered(self.path[randomIndex].state)

            steer1 = Steer(self.path[randomIndex - 1].state, randomState, self.rangeMethod)
            steer2 = Steer(randomState, self.path[randomIndex + 1].state self.rangeMethod)

            if (steer1.isSteerable() and steer2.isSteerable()):
                oldCost = (self.path[randomIndex].cost - self.path[randomIndex - 1].cost) + (self.path[randomIndex + 1].cost - self.path[randomIndex].cost)
                newCost = steer1.getLength() + steer2.getLength()
                if newCost < oldCost:
                    self.path[randomIndex] = RRTNode(randomState, self.path[randomIndex - 1].cost + steer1.getLength(), self.path[randomIndex - 1])
                    print "Rewired a node"

        print "Done Optimizing"