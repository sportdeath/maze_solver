import numpy as np
from Steer import Steer
from RobotState import RobotState

class RRT_Node:
    def __init__(self, state, cost, parent):
        self.state = state      # state is a RobotState
        self.cost = cost
        self.parent = parent

class RRT_Tree:
    def __init__(self, map):
        self.map = map      # map consists of a list of numpy arrays of [x. y] coordinates
        self.tree = None

    # Inputs:
    # init, goal are RobotState objects
    # map 
    def RRT(self, init, goal):
        self.tree = np.array([RRT_Node(init, 0, None)])
        newState = True

        # Building paths to goal
        while(True):
            # Check whether last state we added has steerable path to goal
            if newState:
                steer = Steer(self.tree[-1], goal)
                if (steer.steerable()):
                    # add goal as last node in list
                    self.tree.append(RRT_Node(goal, steer.length + self.tree[-1].cost, self.tree[-1]))
                    break
                newState = False

            randomIndex = np.random.randint(len(map))
            randomPosition = self.map[randomIndex]
            randomTheta = numpy.random.uniform(0, 2*np.pi)
            randomState = RobotState(randomPosition, randomTheta)

            minCost = np.inf
            minNode = None

            for node in self.tree:
                steer = Steer(node.state, randomState)
                if (steer.steerable()):
                    if ((steer.length + node.cost) < minCost):
                        minCost = steer.length + node.cost
                        minNode = node

            # If there is no steerable path, minNode will be None
            if minNode:
                newNode = RRT_Node(randomState, minCost, minNode)
                self.tree.append(newNode)
                newState = True

        # goal node is always last node in self.tree when we break
        path = np.array([self.tree[-1]])
        
        currentNode = self.tree[-1]

        # Back tracking to get path to goal from init
        while (currentNode.parent is not None):
            path.append(currentNode.parent)
            currentNode = currentNode.parent

        # reverse path
        path = path.flipud(path)

        # TODO: spend some time greedily optimizing path
