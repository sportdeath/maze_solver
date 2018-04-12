#!/usr/bin/env python2

import threading
import numpy as np

from rtree import index as rtree_index

import rospy
from rospy.numpy_msg import numpy_msg
from nav_msgs.msg import OccupancyGrid

from maze_solver.rrt.rrt_node import RRTNode
from maze_solver.rrt.rrt_visualizer import RRTVisualizer

class RRT:

    NUM_NEAREST = 20
    MAX_RADIUS = 20.
    MIN_TURNING_RADIUS = 1.
    CARTOGAPHER_MAP_DILATED_TOPIC = "/cartographer_map_dilated"

    def __init__(self):
        # Initialize the tree structure
        self.nodes = {}
        self.node_index = 0
        self.rtree = rtree_index.Index()
        self.rtree_lock = threading.Lock()

        # Precompute gamma for near
        self.gamma_rrt = 2.*(1 + 1/2.)**(1/2.)*self.MAX_RADIUS
        
        # Make a root and a goal
        self.root = RRTNode(
                pose=np.zeros(3),
                cost=0)
        self.insert(self.root)
        self.goal = RRTNode(
                pose=np.zeros(3),
                cost=np.inf)

        # Subscribe to the dilated map
        self.map_msg = None
        self.map_sub = rospy.Subscriber(
                self.CARTOGAPHER_MAP_DILATED_TOPIC,
                numpy_msg(OccupancyGrid),
                self.map_cb, 
                queue_size=1)

        # Make a visualizer
        self.visualizer = RRTVisualizer()

        # Wait for the first map message
        r = rospy.Rate(10)
        while (self.map_msg is None) and (not rospy.is_shutdown()):
            r.sleep()

        # Build the tree
        while not rospy.is_shutdown():
            self.rtree_lock.acquire()
            self.iterate()
            self.rtree_lock.release()
            # if self.node_index % 100 == 0:
                # self.visualizer.visualize_tree(self)

    def map_cb(self, msg):
        """
        Store and reshape the map.

        Args:
            msg: A ROS numpy_msg(OccupancyGrid) message.
        """
        msg.data.shape = (msg.info.height, msg.info.width)
        self.map_msg = msg

    def insert(self, rrt_node):
        """
        Insert a node into the tree.

        Args:
            rrt_node: An RRTNode.
        """
        self.nodes[self.node_index] = rrt_node
        self.rtree.insert(self.node_index,
                (rrt_node.pose[0],
                 rrt_node.pose[1],
                 rrt_node.pose[0],
                 rrt_node.pose[1]))
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
        near = self.rtree.intersection((
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
            if not steer.intersects(self.map_msg):

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
            x_rand.set_parent(x_min, cost_min)

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
            if not steer.intersects(self.map_msg):

                # If the steer does not intersect, compute the path length
                total_cost = steer.length() + x_rand.total_cost()

                # Rewire any node whose path would be shorter if
                # it went through x_rand.
                if total_cost < x_near.total_cost():
                    x_near.set_parent(x_rand, steer.length())

    def iterate(self):
        """
        Perform the RRT* algorithm.
        """

        # Choose a random sample.
        x_rand = RRTNode(self.MAX_RADIUS)

        # Choose the K-nearest neighbors of the random sample
        X_near = self.near(x_rand)

        self.grow_tree(x_rand, X_near)
        if x_rand.parent is not None:
            self.rewire(x_rand, X_near)

            # Check if x_rand is the goal
            # if x_rand.radius() > self.MAX_RADIUS:
                # if x_rand.total_cost() < self.goal.total_cost():
                    # self.goal = x_rand
                    # print "cost: ", self.goal.total_cost(), "radius: ", self.goal.radius()

if __name__ == "__main__":
    rospy.init_node("rrt")
    RRT()
    rospy.spin()
