#!/usr/bin/env python2

import numpy as np

from rtree import index as rtree_index

import rospy
from rospy.numpy_msg import numpy_msg
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker

from maze_solver.steer.steer import DubinsSteer
from maze_solver.utils.visualization_utils import VisualizationUtils

class RRT:

    NUM_NEAREST = 10
    MAX_RADIUS = 30.
    MIN_TURNING_RADIUS = 1.
    PATH_VIZ_TOPIC = "/path_viz"
    CARTOGRAPHER_FRAME = "cartographer_map"

    def __init__(self):
        self.rtree = rtree_index.Index()

        self.root = RRTNode(self.MAX_RADIUS)
        self.root.pose = np.array([0., 0., 0.])
        self.root.cost = 0
        self.insert(self.root)

        self.goal = RRTNode(self.MAX_RADIUS)
        self.goal.pose = np.array([0., 0., 0.])
        self.goal.cost = np.inf

        # Publish the goal path
        self.path_viz_pub = rospy.Publisher(
                self.PATH_VIZ_TOPIC,
                Marker,
                queue_size=1)

        self.map_msg = None
        self.map_sub = rospy.Subscriber(
                "/cartographer_map_dilated",
                numpy_msg(OccupancyGrid),
                self.map_cb, 
                queue_size=1)

        self.num_rewires = 0
        self.num_inserts = 0

        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.map_msg is not None:
                self.iterate()
                self.visualize()
            else:
                r.sleep()

    def map_cb(self, msg):
        msg.data.shape = (msg.info.height, msg.info.width)
        self.map_msg = msg

    def insert(self, rrt_node):
        self.rtree.insert(0,
                (rrt_node.pose[0],
                 rrt_node.pose[1],
                 rrt_node.pose[0],
                 rrt_node.pose[1]),
                obj=rrt_node)

    def nearest(self, rrt_node):
        near = self.rtree.nearest((
            rrt_node.pose[0], 
            rrt_node.pose[1], 
            rrt_node.pose[0], 
            rrt_node.pose[1]), 
            self.NUM_NEAREST,
            objects=True)

        return [n.object for n in near]

    def grow_tree(self, x_rand, X_near):
        cost_min = np.inf
        x_min = None

        for x_near in X_near:

            steer = DubinsSteer(x_near.pose, x_rand.pose, self.MIN_TURNING_RADIUS)
            if not steer.intersects(self.map_msg):

                # If it is compute the length
                cost = steer.length() + x_near.cost
                
                # Update the minimum
                if cost < cost_min:
                    cost_min = cost
                    x_min = x_near

        # Add the min to the tree
        if x_min is not None:
            x_min.add_child(x_rand, cost_min)
            self.insert(x_rand)
            self.num_inserts += 1
            # print "inserts: ", self.num_inserts, "rewires: ", self.num_rewires

    def rewire(self, x_rand, X_near):
        for x_near in X_near:

            steer = DubinsSteer(x_rand.pose, x_near.pose, self.MIN_TURNING_RADIUS)
            if not steer.intersects(self.map_msg):

                cost = steer.length() + x_rand.cost

                if cost < x_near.cost:
                    x_near.set_parent(x_rand, cost)
                    self.num_rewires += 1

    def iterate(self):
        # Choose a random sample
        x_rand = RRTNode(self.MAX_RADIUS)

        # Choose the state k nearest to the random sample
        X_near = self.nearest(x_rand)

        self.grow_tree(x_rand, X_near)
        if x_rand.parent is not None:
            self.rewire(x_rand, X_near)

            if x_rand.radius() > self.MAX_RADIUS:
                if x_rand.cost < self.goal.cost:
                    self.goal = x_rand
                    print "cost: ", self.goal.cost, "radius: ", self.goal.radius()

    def visualize(self):
        if self.path_viz_pub.get_num_connections() > 0:
            if self.goal.parent is None:
                return
            points = []
            node = self.goal
            while node.parent is not None:
                steer = DubinsSteer(node.parent.pose, node.pose, self.MIN_TURNING_RADIUS)
                points.append(steer.sample(0.1))
                node = node.parent
            points = np.concatenate(points, axis=0)

            VisualizationUtils.plot(
                    points[:,0], points[:,1],
                    self.path_viz_pub, 
                    frame=self.CARTOGRAPHER_FRAME,
                    marker_type=Marker.POINTS)

class RRTNode:
    def __init__(self, max_radius):
        # Initialize a node with a random state
        r = np.random.uniform(max_radius + 1)
        phi = np.random.uniform(2*np.pi)
        x = r * np.cos(phi)
        y = r * np.sin(phi)
        theta = np.random.uniform(2*np.pi)

        self.pose = np.array([x, y, theta])

        self.parent = None
        self.children = []

    def set_parent(self, parent, cost):
        if self.parent is not None:
            self.parent.children.remove(self)
        self.parent = parent
        self.cost = cost

    def add_child(self, child, cost):
        self.children.append(child)
        child.parent = self
        child.cost = cost

    def radius(self):
        return np.linalg.norm(self.pose[:2])

if __name__ == "__main__":
    rospy.init_node("rrt")
    RRT()
    rospy.spin()
