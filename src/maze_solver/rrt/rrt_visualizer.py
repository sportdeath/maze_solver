import numpy as np

import rospy
from visualization_msgs.msg import Marker

from maze_solver.utils.visualization_utils import VisualizationUtils

class RRTVisualizer:
    PATH_VIZ_TOPIC = "/path_viz"
    TREE_VIZ_FOR_TOPIC = "/tree_viz_for"
    TREE_VIZ_REV_TOPIC = "/tree_viz_rev"
    CARTOGRAPHER_FRAME = rospy.get_param("/maze_solver/cartographer_frame")
    OCCUPIED_TOPIC = "/occupied"

    def __init__(self):
        # Publish the goal path
        self.occupied_pub = rospy.Publisher(
                self.OCCUPIED_TOPIC,
                Marker,
                queue_size=1)
        self.path_viz_pub = rospy.Publisher(
                self.PATH_VIZ_TOPIC,
                Marker,
                queue_size=1)
        self.tree_viz_for_pub = rospy.Publisher(
                self.TREE_VIZ_FOR_TOPIC,
                Marker,
                queue_size=1)
        self.tree_viz_rev_pub = rospy.Publisher(
                self.TREE_VIZ_REV_TOPIC,
                Marker,
                queue_size=1)

    def visualize_changed_points(self, occupied_points):
        if self.occupied_pub.get_num_connections() > 0:
            VisualizationUtils.plot(
                    occupied_points[:,0],
                    occupied_points[:,1],
                    self.occupied_pub, 
                    frame=self.CARTOGRAPHER_FRAME,
                    marker_type=Marker.POINTS)

    def visualize_path(self, path):
        if self.path_viz_pub.get_num_connections() > 0:
            points = []
            for steer, reverse in path:
                samples = steer.sample(0.05)
                if reverse:
                    samples = samples[::-1]
                points.append(samples)

            points = np.concatenate(points, axis=0)
            VisualizationUtils.plot(
                    points[:,0], points[:,1],
                    self.path_viz_pub, 
                    color=(0.42,0.835,0.35),
                    frame=self.CARTOGRAPHER_FRAME)

    def visualize_tree(self, rrt):
        if self.tree_viz_for_pub.get_num_connections() > 0:
            forward_points = []
            reverse_points = []
            for i, node in rrt.nodes.iteritems():
                if node.parent is None:
                    continue
                steer = node.steer(rrt.MIN_TURNING_RADIUS)
                samples = steer.sample(0.2)
                if node.reverse:
                    forward_points.append(samples)
                else:
                    reverse_points.append(samples)
            if len(forward_points) > 0:
                points = np.concatenate(forward_points, axis=0)
                VisualizationUtils.plot(
                        points[:,0], points[:,1],
                        self.tree_viz_for_pub, 
                        color=(0., 0., 1.),
                        frame=self.CARTOGRAPHER_FRAME,
                        marker_type=Marker.POINTS)
            if len(reverse_points) > 0:
                points = np.concatenate(reverse_points, axis=0)
                VisualizationUtils.plot(
                        points[:,0], points[:,1],
                        self.tree_viz_rev_pub, 
                        color=(1., 0., 0.),
                        frame=self.CARTOGRAPHER_FRAME,
                        marker_type=Marker.POINTS)
