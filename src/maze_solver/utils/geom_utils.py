import numpy as np

import tf
from geometry_msgs.msg import Quaternion

class GeomUtils:
    """
    Useful methods for geometry in 2D.
    """

    @staticmethod
    def quaternion_to_angle(q):
        """
        Convert a quaternion ROS message into an angle in radians.
        """
        x, y, z, w = q.x, q.y, q.z, q.w
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((x, y, z, w))

        return yaw

    @staticmethod
    def angle_to_quaternion(angle):
        """
        Convert an angle in radians into a quaternion ROS message.
        """
        return Quaternion(*tf.transformations.quaternion_from_euler(0, 0, angle))

    @staticmethod
    def get_angle(vector):
        """
        Get the angle of a 2D vector in the plane.
        An angle of 0 rad means the vector is pointing
        in the positive x direction and the angle
        increases and the vector rotates counterclockwise
        """

        # Normalize the vector
        vector = vector/np.linalg.norm(vector)

        return np.arccos(vector[0]) * np.sign(vector[1])

    @staticmethod
    def get_vector(angle):
        """
        Produce a unit vector in 2D from an angle.
        """
        return np.array([np.cos(angle), np.sin(angle)])

    @staticmethod
    def getAngleBetweenVectors(init, goal, direction):
        initAngle = -direction * GeomUtils.get_angle(init)
        goalAngle = -direction * GeomUtils.get_angle(goal)
        if goalAngle < initAngle:
            goalAngle += 2 * np.pi

        return goalAngle - initAngle

    @staticmethod
    def get_perpendicular(norm):
        """
        Get a vector that is perpendicular
        to the input vector.
        """
        return np.array([norm[1], -norm[0]])

    @staticmethod
    def rotate_vector(vector, angle):
        """
        Rotate the input vector by the angle.
        """
        sin_angle = np.sin(angle)
        cos_angle = np.cos(angle)
        rotation_matrix = np.array(
                [[cos_angle, -sin_angle],
                 [sin_angle, cos_angle]])
        return np.dot(rotation_matrix, vector)
