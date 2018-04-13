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

    # Fetch parameters
    P_UNIFORM = rospy.get_param("/maze_solver/p_uniform")
    BRIDGE_STD_DEV = rospy.get_param("/maze_solver/bridge_std_dev")
    DILATION_RADIUS = rospy.get_param("/maze_solver/dilation_radius")
    OCCUPANCY_THRESHOLD = rospy.get_param("/maze_solver/occupancy_threshold")
    CAR_RADIUS = rospy.get_param("/maze_solver/car_radius")
    SEARCH_RADIUS = rospy.get_param("/maze_solver/search_radius")
    MIN_TURNING_RADIUS = rospy.get_param("/maze_solver/min_turning_radius")
    CARTOGAPHER_MAP_DILATED_TOPIC = rospy.get_param("maze_solver/cartographer_dilated_topic")

    def __init__(self, pose=np.zeros(3)):
        # Initialize the tree structure
        self.nodes = {}
        self.node_index = 0
        self.rtree = rtree_index.Index()
        self.rtree_lock = threading.Lock()

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
        self.goal = None
        self.goal_cost = np.inf

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
            self.visualizer.visualize_path(self)
            self.visualizer.visualize_tree(self)
            if self.node_index % 100 == 0:
                print self.node_index

    def map_cb(self, msg):
        """
        Store and reshape the map.

        Args:
            msg: A ROS numpy_msg(OccupancyGrid) message.
        """
        msg.data.shape = (msg.info.height, msg.info.width)
        self.occupied_points = np.argwhere(msg.data > self.OCCUPANCY_THRESHOLD)
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
            if not steer.intersects(self.sample_width, self.map_msg, self.OCCUPANCY_THRESHOLD):

                # If the steer does not intersect, compute the path length
                total_cost = steer.length() + x_rand.total_cost()

                # Rewire any node whose path would be shorter if
                # it went through x_rand.
                if total_cost < x_near.total_cost():
                    x_near.set_parent(x_rand, steer.length())

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

            # If there is no intersection check the total cost
            total_cost = steer.length() + node.total_cost()
            if total_cost < self.goal_cost:
                self.insert(closest)
                closest.set_parent(node, steer.length())
                self.goal = closest
                self.goal_cost = total_cost

    def iterate(self):
        """
        Perform the RRT* algorithm.
        """

        # Choose a random sample.
        x_rand = RRTNode(
                p_uniform=self.P_UNIFORM,
                max_radius=self.SEARCH_RADIUS,
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

if __name__ == "__main__":
    rospy.init_node("rrt")
    RRT((1., 1., 0.5))
    rospy.spin()
