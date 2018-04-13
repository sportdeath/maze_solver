import threading
import numpy as np

from rtree import index as rtree_index

import rospy
from geometry_msgs.msg import PoseArray

from maze_solver.rrt.rrt_node import RRTNode
from maze_solver.rrt.map_differences import new_occupancies

class RRT:

    # Fetch parameters
    P_UNIFORM = rospy.get_param("/maze_solver/p_uniform")
    BRIDGE_STD_DEV = rospy.get_param("/maze_solver/bridge_std_dev")
    DILATION_RADIUS = rospy.get_param("/maze_solver/dilation_radius")
    OCCUPANCY_THRESHOLD = rospy.get_param("/maze_solver/occupancy_threshold")
    CAR_RADIUS = rospy.get_param("/maze_solver/car_radius")
    SEARCH_RADIUS = rospy.get_param("/maze_solver/search_radius")
    MIN_TURNING_RADIUS = rospy.get_param("/maze_solver/min_turning_radius")

    def __init__(self, pose, map_msg):
        self.map_msg = map_msg
        self.occupied_points = np.argwhere(self.map_msg.data > self.OCCUPANCY_THRESHOLD)

        # Initialize the tree structure
        self.nodes = {}
        self.node_index = 0
        self.rrt_tree = rtree_index.Index()
        self.lock = threading.Lock()

        # Precompute gamma for near
        self.gamma_rrt = 2.*(1 + 1/2.)**(1/2.)*self.SEARCH_RADIUS

        # Compute the sample width necessary for the car
        # not to intersect with obstacles
        self.sample_width = 2 * np.sqrt(self.DILATION_RADIUS**2 - self.CAR_RADIUS**2)
        
        # Make a root and a goal
        self.root = RRTNode(
                pose=pose,
                cost=0)
        self.insert(self.root)
        self.check_goal(self.root)
        self.goal_nodes = []

    def insert(self, rrt_node):
        """
        Insert a node into the tree.

        Args:
            rrt_node: An RRTNode.
        """
        self.nodes[self.node_index] = rrt_node
        self.rrt_tree.insert(self.node_index,
                (rrt_node.pose[0],
                 rrt_node.pose[1],
                 rrt_node.pose[0],
                 rrt_node.pose[1]))
        rrt_node.index = self.node_index
        self.node_index += 1

    def near(self, rrt_node):
        """
        Get the K-nearest neighbors of rrt_node.

        Args:
            rrt_node: An RRTNode.

        Returns:
            A list of RRTNodes.
        """
        near_radius = self.gamma_rrt * (np.log(len(self.nodes)+1)/(len(self.nodes)+1))**(1/2.)
        near = self.rrt_tree.intersection((
            rrt_node.pose[0] - near_radius,
            rrt_node.pose[1] - near_radius,
            rrt_node.pose[0] + near_radius,
            rrt_node.pose[1] + near_radius))

        return [self.nodes[i] for i in near]

    def grow_tree(self, x_rand, X_near):
        """
        Grow the tree according to the RRT* algorithm.

        Args:
            x_rand: A random RRTNode sample.
            X_near: The K-nearest neighbors of x_rand.
                A list of RRTNodes.
        """

        total_cost_min = np.inf
        cost_min = 0
        x_min = None
        for x_near in X_near:

            # Try steering from x_near to x_rand
            steer = x_rand.steer(self.MIN_TURNING_RADIUS, parent=x_near)
            if not steer.intersects(self.sample_width, self.map_msg, self.OCCUPANCY_THRESHOLD):

                # If the steer does not intersect, compute the path length
                total_cost = steer.length() + x_near.total_cost()
                
                # Update the minimum cost path to x_rand
                if total_cost < total_cost_min:
                    total_cost_min = total_cost
                    cost_min = steer.length()
                    x_min = x_near

        # Add the minimum cost path to the tree
        if x_min is not None:
            self.insert(x_rand)
            x_rand.set_parent(
                    x_min, 
                    cost_min)

    def rewire(self, x_rand, X_near):
        """
        Rewire the tree according to the RRT* algorithm.

        Args:
            x_rand: A random RRTNode sample.
            X_near: The K-nearest neighbors of x_rand.
                A list of RRTNodes.
        """

        for x_near in X_near:

            # Try steering from x_rand to x_near
            steer = x_near.steer(self.MIN_TURNING_RADIUS, parent=x_rand)
            if not steer.intersects(self.sample_width, self.map_msg, self.OCCUPANCY_THRESHOLD):

                # If the steer does not intersect, compute the path length
                total_cost = steer.length() + x_rand.total_cost()

                # Rewire any node whose path would be shorter if
                # it went through x_rand.
                if total_cost < x_near.total_cost():
                    x_near.set_parent(
                            x_rand,
                            steer.length())

    def check_goal(self, node):
        # Find the closest point on the boundary
        position = node.pose[:2]
        direction = node.pose[:2]/np.linalg.norm(node.pose[:2])
        closest_pose = np.array([
            direction[0] * self.SEARCH_RADIUS,
            direction[1] * self.SEARCH_RADIUS,
            np.arctan2(direction[1], direction[0])])
        closest = RRTNode(pose=closest_pose)

        # Steer from the node to that point
        steer = closest.steer(self.MIN_TURNING_RADIUS, parent=node)
        if not steer.intersects(self.sample_width, self.map_msg, self.OCCUPANCY_THRESHOLD):
            self.insert(closest)
            closest.set_parent(
                    node,
                    steer.length())
            self.goal_nodes.append(closest)

    def path(self):
        """
        Get the shortest path to the goal.
        """
        min_node = None
        min_cost = np.inf
        for goal_node in self.goal_nodes:
            cost = goal_node.total_cost()
            if cost < min_cost:
                min_node = goal_node
                min_cost = cost

        if min_node is None:
            return min_node

        return min_node.path(self.MIN_TURNING_RADIUS)

    def iterate(self):
        """
        Perform the RRT* algorithm.
        """
        # Lock
        self.lock.acquire()

        # Choose a random sample.
        x_rand = RRTNode(
                p_uniform=self.P_UNIFORM,
                search_radius=self.SEARCH_RADIUS,
                bridge_std_dev=self.BRIDGE_STD_DEV,
                map_msg=self.map_msg,
                occupied_points=self.occupied_points,
                occupancy_threshold=self.OCCUPANCY_THRESHOLD)

        # Choose the K-nearest neighbors of the random sample
        X_near = self.near(x_rand)

        self.grow_tree(x_rand, X_near)
        if x_rand.parent is not None:
            self.rewire(x_rand, X_near)
            self.check_goal(x_rand)

        # Unlock
        self.lock.release()

if __name__ == "__main__":
    rospy.init_node("rrt")
    RRT((1., 1., 0.5))
    rospy.spin()