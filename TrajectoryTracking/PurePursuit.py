#!/usr/bin/env python
from Config import *

import numpy as np
import rospy
import tf.transformations
import tf

from nav_msgs.srv import GetMap
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseArray, Quaternion
from ackermann_msgs.msg import AckermannDriveStamped

from TrajectoryTracker import TrajectoryTracker
from utils import Utils


class PurePursuit:

    def __init__(self, path_file_path):
        self.getMap()

        self.poseSub = rospy.Subscriber("/pf/viz/inferred_pose", PoseStamped, self.poseCB, queue_size = 1)
        self.commandPub = rospy.Publisher(\
                "/vesc/high_level/ackermann_cmd_mux/input/nav_0",\
                AckermannDriveStamped,\
                queue_size = 1)
        self.pathPub = rospy.Publisher("/pp/map_points", PoseArray, queue_size = 1)
        self.nextGoalPub = rospy.Publisher("/pp/next_goal_point", PoseStamped, queue_size = 1)
        
        self.transformationFramePublisher = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()

        self.carLength = CAR_LENGTH

        self.points = np.loadtxt(path_file_path)
        self.trajectoryTracker = TrajectoryTracker(self.points)

        self.x = None
        self.y = None
        self.angle = None

        self.atEndOfPath = False

    def getMap(self):
        mapServiceName = rospy.get_param("~static_map", "static_map")
        rospy.wait_for_service(mapServiceName)
        self.mapMsg = rospy.ServiceProxy(mapServiceName, GetMap)().map
        self.mapInfo = self.mapMsg.info
        print "Map loaded"

    # goalPoint - a tuple (x, y) measured in meters
    # representing the goal point of pure pursuit 
    # relative to the car
    #
    # Returns the angle the car should steer at
    def getControlAngle(self):
        if self.x:
            print "RESETTING CONTROL ANGLE"
            carPosition = np.array([self.x, self.y])
            goalPointWorld = self.trajectoryTracker.getGoalPointGlobal(carPosition)
            self.publish_point(goalPointWorld)

            try:
                (trans,rot) = listener.lookupTransform('/turtle2', '/turtle1', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print "COULD NOT GET TRANSFORM"
                return 0

            transform = np.dot(trans, rot)
            goalPointRobot = np.dot(transform, goalPointWorld)

            relativeGoalPosition = goalPointRobot - carPosition

            goalDistance = np.linalg.norm(relativeGoalPosition)

            # Rotate the relative position by -theta

            goalAlpha = np.arccos(np.dot(relativeGoalPosition, np.array([1, 0]))/goalDistance)
            rospy.loginfo("alpha : " + str(goalAlpha))

            goalAlpha -= self.theta
            rospy.loginfo("alpha (after): " + str(goalAlpha))

            rospy.loginfo("goal distance: " + str(goalDistance))
            rospy.loginfo("look ahead distance: " + str(LOOK_AHEAD_DISTANCE))

            goalPointLocal = np.dot(Utils.rotation_matrix(goalAlpha), relativeGoalPosition)
            sinAlpha = goalPointLocal.item(1)/goalDistance
            curvature = 2*sinAlpha/goalDistance

            return -np.arctan(curvature * self.carLength)

    def poseCB(self, msg):
        self.x = msg.pose.position.x    # map coordinates
        self.y = msg.pose.position.y
        self.theta = Utils.quaternion_to_angle(msg.pose.orientation)

        steeringAngle = self.getControlAngle()
        msg = AckermannDriveStamped()
        msg.header = Utils.make_header("velocity")

        if not self.atEndOfPath and self.trajectoryTracker.atEndOfPath([self.x, self.y]):
            self.atEndOfPath = True
            print "FINISHED TRAVERSING PATH"

        if not self.atEndOfPath:
            msg.drive.speed = CAR_VELOCITY
            msg.drive.steering_angle = steeringAngle
        else:
            msg.drive.speed = 0
            msg.drive.steering_angle = 0
            print "FINISHED PATH"

        print 'steering angle', steeringAngle
        self.publish_points()
        self.commandPub.publish(msg)

    def publish_point(self, point):
        print "PUBLISHING NEW POINT"
        pose = Utils.particle_to_pose(np.concatenate((point,[0])))
        ps = PoseStamped()
        ps.header = Utils.make_header("map")
        ps.pose = pose
        self.nextGoalPub.publish(ps)
        self.publishTransformationFrame()

    def publish_points(self):
        self.mapPoints = np.copy(self.points)
        zeroThetas = np.zeros((self.points.shape[0],1))
        particles = np.hstack((self.mapPoints, zeroThetas))

        pa = PoseArray()
        pa.header = Utils.make_header("map")
        pa.poses = Utils.particles_to_poses(particles)

        self.pathPub.publish(pa)
        self.publishTransformationFrame()

    def publishTransformationFrame(self):
        """ Publish a tf from map to base_link. """
        stamp = rospy.Time.now()

        position = (0, 0, 0)
        orientation = (0,0,0,1)

        self.transformationFramePublisher.sendTransform(
                position, 
                orientation, 
                stamp, 
                "/base_link", 
                "/map");


if __name__=="__main__":
    rospy.init_node("PurePursuit")

    pp = PurePursuit(PATH_PLANNER_OUTPUT_FIXED)
    rospy.spin()
