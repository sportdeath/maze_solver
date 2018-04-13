#!/usr/bin/env python2

from timeit import default_timer as timer

import numpy as np

import rospy
import tf.transformations
from rospy.numpy_msg import numpy_msg
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, PoseArray

from maze_solver.rrt.pose_sampler import PoseSampler
from maze_solver.trajectory_tracking.trajectory_tracker import TrajectoryTracker

class TestPoseSampler:

    SAMPLE_TOPIC = "/sample_test"
    CARTOGRAPHER_DILATED_TOPIC = rospy.get_param("/maze_solver/cartographer_dilated_topic")
    CARTOGRAPHER_FRAME = rospy.get_param("/maze_solver/cartographer_frame")
    OCCUPANCY_THRESHOLD = rospy.get_param("/maze_solver/occupancy_threshold")

    def __init__(self):
        # Trajectory Tracker (for pose)
        self.pose = None
        self.tracker = TrajectoryTracker()

        # The pose sampler itself
        self.pose_sampler = PoseSampler()

        # Publish a pose array
        self.pose_pub = rospy.Publisher(
                self.SAMPLE_TOPIC,
                PoseArray,
                queue_size=1)

        # Subscribe to the dilated map
        self.map_msg = None
        self.map_sub = rospy.Subscriber(
                self.CARTOGRAPHER_DILATED_TOPIC,
                numpy_msg(OccupancyGrid),
                self.map_cb, 
                queue_size=1)

        # Wait for the first map message
        r = rospy.Rate(10)
        while ((self.map_msg is None) or (self.pose is None)) and (not rospy.is_shutdown()):
            self.pose = self.tracker.pose
            r.sleep()


        # Sample random points and publish them
        pose_msg = PoseArray()
        pose_msg.header.frame_id = self.CARTOGRAPHER_FRAME
        n = 0
        s = 0
        while not rospy.is_shutdown():
            # Sample a random pose and time it
            start = timer()
            # pose = self.pose_sampler.bridge(self.map_msg, self.occupied_points)
            # pose = PoseSampler.uniform(20., self.map_msg, self.OCCUPANCY_THRESHOLD)
            pose = self.pose_sampler.hybrid(
                    self.pose,
                    self.map_msg,
                    self.occupied_points)
            end = timer()
            s += (end - start)
            n += 1
            if n % 10 == 0:
                print "Average running time: ", s/n

            # Add the pose to the list of poses
            p = Pose()
            p.position.x = pose[0]
            p.position.y = pose[1]
            quat = tf.transformations.quaternion_from_euler(0, 0, pose[2])
            p.orientation.x = quat[0]
            p.orientation.y = quat[1]
            p.orientation.z = quat[2]
            p.orientation.w = quat[3]
            pose_msg.poses.append(p)

            self.pose_pub.publish(pose_msg)

    def map_cb(self, msg):
        """
        Store and reshape the map.

        Args:
            msg: A ROS numpy_msg(OccupancyGrid) message.
        """
        msg.data.shape = (msg.info.height, msg.info.width)
        self.occupied_points = np.argwhere(msg.data > self.OCCUPANCY_THRESHOLD)
        self.map_msg = msg

if __name__ == "__main__":
    rospy.init_node("test_pose_sampler")
    TestPoseSampler()
    rospy.spin()
