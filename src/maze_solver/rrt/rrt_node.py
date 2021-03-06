import numpy as np

from maze_solver.rrt.steer import DubinsSteer

class RRTNode:
    def __init__(self, 
            pose=None, 
            pose_sampler=None,
            car_pose=None,
            map_msg=None,
            occupied_points=None,
            cost=None,
            p_reverse=None):
        """
        Create a node to be placed in an RRT* tree.

        Args:
            search_radius: The radius away from the origin that
                poses are generated within.
            poas: The pose of the node. If pose is set to None,
                the pose is randomly generated. The position of
                the pose is generated uniformly at random within
                search_radius and orientation is uniform in
                [-Pi, Pi).
            cost: The cost of the node measured as the distance
                from the parent node.
            reverse: If true, the path from this node to the parent
                node is driven in reverse.
        """


        if pose is None:
            # Initialize a node with a random state
            self.pose = pose_sampler.hybrid(
                    car_pose,
                    map_msg,
                    occupied_points)
        else:
            self.pose = pose

        if p_reverse is not None:
            self.reverse = (np.random.uniform() < p_reverse)
        else:
            self.reverse = False

        self.cost = cost

        self.parent = None
        self.children = []

    def steer(self, min_turning_radius, parent=None):
        """
        Determine a steering function from the parent
        to self.
        
        Args:
            min_turning_radius: The minimum turning radius
                of the vehicle.
            parent: The parent of the node. If none is specified
                the node uses its  own parent.

        Returns:
            A Steer object.
        """
        if parent is None:
            parent = self.parent
        start=parent.pose
        end=self.pose
        if self.reverse:
            start, end = end, start
        return DubinsSteer(start, end, min_turning_radius)

    def total_cost(self):
        """
        Returns the total cost to go from the
        root node to self.
        """
        c = 0
        node = self
        while node is not None:
            c += node.cost
            node = node.parent
        return c

    def set_parent(self, parent, cost):
        """
        Sets the parent of a node and adds self as
        a child of the parent node.

        Args:
            parent: The desired parent, an RRTNode.
            cost: The cost of moving from parent
                to self.
        """
        if self.parent is not None:
            self.parent.children.remove(self)
        self.parent = parent
        self.parent.children.append(self)
        self.cost = cost

    def path(self, min_turning_radius):
        node = self
        path = []
        while node.parent is not None:
            steer = node.steer(min_turning_radius)
            path.append((steer, node.reverse))
            node = node.parent

        return path[::-1]
