#!/usr/bin/env python2

from timeit import default_timer as timer

import numpy as np

import rospy
import tf.transformations
from rospy.numpy_msg import numpy_msg
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, PoseArray

from maze_solver.rrt.pose_sampler import PoseSampler

class TestPoseSampler:

    CARTOGAPHER_MAP_DILATED_TOPIC = "/cartographer_map_dilated"
    SAMPLE_TOPIC = "/sample_test"
    CARTOGRAPHER_FRAME = "cartographer_map"
    OCCUPANCY_THRESHOLD = 80
    P_UNIFORM = 0.3
    MAX_RADIUS = 20.
    BRIDGE_STD_DEV = 1.

    def __init__(self):
        # Subscribe to the dilated map
        self.map_msg = None
        self.map_sub = rospy.Subscriber(
                self.CARTOGAPHER_MAP_DILATED_TOPIC,
                numpy_msg(OccupancyGrid),
                self.map_cb, 
                queue_size=1)

        self.pose_pub = rospy.Publisher(
                self.SAMPLE_TOPIC,
                PoseArray,
                queue_size=1)

        # Wait for the first map message
        r = rospy.Rate(10)
        while (self.map_msg is None) and (not rospy.is_shutdown()):
            r.sleep()


        # Sample random points and publish them
        pose_msg = PoseArray()
        pose_msg.header.frame_id = self.CARTOGRAPHER_FRAME
        n = 0
        s = 0
        while not rospy.is_shutdown():
            # Sample a random pose and time it
            start = timer()
            # pose = PoseSampler.bridge(1., self.map_msg, self.occupied_points, self.OCCUPANCY_THRESHOLD)
            # pose = PoseSampler.uniform(20., self.map_msg, self.OCCUPANCY_THRESHOLD)
            pose = PoseSampler.hybrid(
                    self.P_UNIFORM,
                    self.MAX_RADIUS,
                    self.BRIDGE_STD_DEV,
                    self.map_msg,
                    self.occupied_points,
                    self.OCCUPANCY_THRESHOLD)
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
        self.map_msg = msg
        self.occupied_points = np.argwhere(msg.data > self.OCCUPANCY_THRESHOLD)

if __name__ == "__main__":
    rospy.init_node("test_pose_sampler")
    TestPoseSampler()
    rospy.spin()
