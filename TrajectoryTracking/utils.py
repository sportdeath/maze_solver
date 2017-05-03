#!/usr/bin/env python

'''
- helper functions for particle filter:
    - coordinate space conversions
    - useful ROS object construction functions
'''

import rospy
import numpy as np

from std_msgs.msg import Header
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Quaternion, Point32
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetMap
import tf.transformations
import tf

class Utils(object):
    """ Helper functions """
        
    @staticmethod
    def angle_to_quaternion(angle):
        """Convert an angle in radians into a quaternion _message_."""
        return Quaternion(*tf.transformations.quaternion_from_euler(0, 0, angle))

    @staticmethod
    def quaternion_to_angle(q):
        """Convert a quaternion _message_ into an angle in radians.
        The angle represents the yaw.
        This is not just the z component of the quaternion."""
        x, y, z, w = q.x, q.y, q.z, q.w
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((x, y, z, w))
        return yaw

    @staticmethod
    # gives a rotation matrix for rotating coordinates by theta
    # not recommended for direct use since this will be slow
    # instead apply the same math over a whole array all at once when possible
    def rotation_matrix(theta):
        c, s = np.cos(theta), np.sin(theta)
        return np.matrix([[c, -s], [s, c]])

    @staticmethod
    def particle_to_pose(particle):
        pose = Pose()
        pose.position.x = particle[0]
        pose.position.y = particle[1]
        pose.orientation = Utils.angle_to_quaternion(particle[2])
        return pose

    @staticmethod
    def particles_to_poses(particles):
        return map(Utils.particle_to_pose, particles)

    # @staticmethod
    # def particle_to_marker(particle, weight):
    #     pose = Pose()
    #     pose.position.x = particle[0]
    #     pose.position.y = particle[1]
    #     pose.orientation = Utils.angle_to_quaternion(particle[2])
    #     marker = Marker()
    #     marker.type = 2
    #     marker.pose = pose
    #     marker.size = weight
    #     return marker

    # @staticmethod
    # def particles_to_poses(particles, weights):
    #     return map(Utils.particle_to_marker, particles, weights)

    @staticmethod
    def make_header(frame_id, stamp=None):
        if stamp == None:
            stamp = rospy.Time.now()
        header = Header()
        header.stamp = stamp
        header.frame_id = frame_id
        return header

    @staticmethod
    def point(npt):
        pt = Point32()
        pt.x = npt[0]
        pt.y = npt[1]
        return pt

    @staticmethod
    def points(arr):
        return map(Utils.point, arr)

    @staticmethod
    def make_circle_marker(point, scale, color, frame_id, namespace, sid, duration=0):
        marker = Marker()
        marker.header = Utils.make_header(frame_id)
        marker.ns = namespace
        marker.id = sid
        marker.type = 2 # sphere
        marker.lifetime = rospy.Duration.from_sec(duration)
        marker.action = 0
        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.pose.orientation.w = 1.0
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color.r = float(color[0])
        marker.color.g = float(color[1])
        marker.color.b = float(color[2])
        marker.color.a = 1.0
        return marker

    @staticmethod
    def marker_clear_all(frame_id):
        # Create a marker which clears all.
        marker = Marker()
        marker.header.frame_id = frame_id;
        marker.action = 3 # DELETEALL action.
        return marker

    # the following functions are for converting to/from map and world coordinates
    # useful for converting from world poses to array indices in the "self.permissible" array
    # they may not be exactly what you need, and you may neet to switch your
    # x and y coorindates before indexing into the permissible array

    # converts map space coordinates to world space coordinates
    # this version is slow but easy to follow logically
    @staticmethod
    def map_to_world_slow(x,y,t,map_info):
        scale = map_info.resolution
        angle = Utils.quaternion_to_angle(map_info.origin.orientation)
        rot = Utils.rotation_matrix(angle)
        trans = np.array([[map_info.origin.position.x],
                          [map_info.origin.position.y]])

        map_c = np.array([[x],
                          [y]])
        world = (rot*map_c) * scale + trans

        return world[0,0],world[1,0],t+angle

    # converts world space coordinates to map space coordinates
    @staticmethod
    def world_to_map_slow(x,y,t, map_info):
        scale = map_info.resolution
        angle = Utils.quaternion_to_angle(map_info.origin.orientation)
        rot = Utils.rotation_matrix(-angle)
        trans = np.array([[map_info.origin.position.x],
                          [map_info.origin.position.y]])

        world = np.array([[x],
                          [y]])
        map_c = rot*((world - trans) / float(scale))
        return map_c[0,0],map_c[1,0],t-angle

    @staticmethod
    # same as above but faster, operates on an Nx3 array of poses
    def map_to_world(poses,map_info):
        scale = map_info.resolution
        angle = Utils.quaternion_to_angle(map_info.origin.orientation)

        # rotation
        c, s = np.cos(angle), np.sin(angle)
        # we need to store the x coordinates since they will be overwritten
        temp = np.copy(poses[:,0])
        poses[:,0] = c*poses[:,0] - s*poses[:,1]
        poses[:,1] = s*temp       + c*poses[:,1]

        # scale
        poses[:,:2] *= float(scale)

        # translate
        poses[:,0] += map_info.origin.position.x
        poses[:,1] += map_info.origin.position.y
        poses[:,2] += angle
        
    @staticmethod
    # same as above but faster, operates on an Nx3 array of poses
    def world_to_map(poses, map_info):
        # operates in place
        scale = map_info.resolution
        angle = -Utils.quaternion_to_angle(map_info.origin.orientation)

        # translation
        poses[:,0] -= map_info.origin.position.x
        poses[:,1] -= map_info.origin.position.y

        # scale
        poses[:,:2] *= (1.0/float(scale))

        # rotation
        c, s = np.cos(angle), np.sin(angle)
        # we need to store the x coordinates since they will be overwritten
        temp = np.copy(poses[:,0])
        poses[:,0] = c*poses[:,0] - s*poses[:,1]
        poses[:,1] = s*temp       + c*poses[:,1]
        poses[:,2] += angle
