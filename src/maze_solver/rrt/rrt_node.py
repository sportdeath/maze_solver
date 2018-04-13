import numpy as np

from maze_solver.rrt.steer import DubinsSteer
from maze_solver.rrt.pose_sampler import PoseSampler

class RRTNode:
    def __init__(self, 
            pose=None, 
            p_uniform=None,
            bridge_std_dev=None,
            max_radius=None,
            map_msg=None,
            occupied_points=None,
            occupancy_threshold=None,
            cost=None, 
            reverse=None):
        """
        Create a node to be placed in an RRT* tree.

        Args:
            max_radius: The radius away from the origin that
                poses are generated within.
            poas: The pose of the node. If pose is set to None,
                the pose is randomly generated. The position of
                the pose is generated uniformly at random within
                max_radius and orientation is uniform in
                [-Pi, Pi).
            cost: The cost of the node measured as the distance
                from the parent node.
            reverse: If true, the path from this node to the parent
                node is driven in reverse.
        """

        self.reverse = False

        if pose is None:
            # Initialize a node with a random state
            self.pose = PoseSampler.hybrid(
                    p_uniform,
                    max_radius,
                    bridge_std_dev,
                    map_msg,
                    occupied_points,
                    occupancy_threshold)
        else:
            self.pose = pose

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

    def radius(self):
        return np.linalg.norm(self.pose[:2])

    def set_root(self, tree_lock):
        """
        Set a node to the root of the tree.

        Args:
            tree_lock: A threading.Lock() object, required
                to avoid threading conflicts with a growing
                RRT* tree.
        """

        tree_lock.aquire()

        cost = self.cost
        parent = self.parent
        self.cost = 0
        self.parent = None

        # parent.set_parent(
        # parent.reverse = True
        # parent.cost = cost

        # Set the parent's parent to the node
        # and set it to be in reverse
        node.parent.reverse = True
        node.parent.cost = node.cost
        node.parent.set_parent(node)

        # Prevent a loop and make minimum cost
        node.parent = None
        node.cost = 0

        tree_lock.release()
