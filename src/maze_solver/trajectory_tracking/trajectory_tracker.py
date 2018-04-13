import threading

import numpy as np

import rospy
import tf_conversions
import tf2_ros
import tf2_geometry_msgs
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header
from visualization_msgs.msg import Marker

from maze_solver.trajectory_tracking.pure_pursuit import PurePursuit
from maze_solver.utils.visualization_utils import VisualizationUtils

class TrajectoryTracker:
    TRAJECTORY_SAMPLE_WIDTH = rospy.get_param("/maze_solver/trajectory_sample_width")
    TRAJECTORY_RATE = rospy.get_param("/maze_solver/trajectory_rate")
    AXLE_LENGTH = rospy.get_param("/maze_solver/axle_length")
    LOOK_AHEAD_DISTANCE = rospy.get_param("/maze_solver/look_ahead_distance")
    LOOK_AHEAD_DISTANCE_SQUARED = LOOK_AHEAD_DISTANCE * LOOK_AHEAD_DISTANCE
    MAX_STEERING_ANGLE = rospy.get_param("/maze_solver/max_steering_angle")
    DRIVE_TOPIC = rospy.get_param("/maze_solver/drive_topic")
    CARTOGRAPHER_FRAME = rospy.get_param("/maze_solver/cartographer_frame")
    BASE_FRAME = rospy.get_param("/maze_solver/base_frame")
    VELOCITY = rospy.get_param("/maze_solver/velocity")
    LOOK_AHEAD_VIZ_TOPIC = "/look_ahead"
    MIN_VELOCITY = 0.2
    VELOCITY_PROP = 0.5

    def __init__(self):
        self.lock = threading.Lock()

        # Create a publisher
        self.drive_pub = rospy.Publisher(
                self.DRIVE_TOPIC,
                AckermannDriveStamped,
                queue_size=1)
        self.look_ahead_pub = rospy.Publisher(
                self.LOOK_AHEAD_VIZ_TOPIC,
                Marker,
                queue_size=1)

        # Listen to new poses
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Initialize the pose
        self.pose = None

        # Initialize stopped
        self.path = None
        self.is_lost = True
        self.at_end = True

        # Create a drive timer
        rospy.Timer(rospy.Duration(self.TRAJECTORY_RATE), self.drive)

    def stop(self):
        """
        Stop driving.
        """
        self.lock.acquire()
        self.at_end = True
        self.lock.release()
        self.drive()

    def track(self, path):
        self.lock.acquire()
        # Sample the path
        points = []
        self.reverses = []
        for i, (steer, reverse) in enumerate(path):
            samples = steer.sample(self.TRAJECTORY_SAMPLE_WIDTH)
            if reverse:
                samples = samples[::-1]
            points.append(samples)
            self.reverses += [reverse] * len(samples)

            if i + 1 == len(path) or path[i + 1][1] != reverse:
                # We are reversing direction or at the end of the path
                # Add a look ahead point
                offset = np.array([[
                    self.LOOK_AHEAD_DISTANCE * np.cos(samples[-1,2]),
                    self.LOOK_AHEAD_DISTANCE * np.sin(samples[-1,2]),
                    0]])
                if reverse:
                    offset *= -1
                points.append(samples[-1:,:] + offset)

                self.reverses += [self.reverses[-1]]

        self.path_index = 0
        self.path = np.concatenate(points, axis=0)[:, :2]
        self.is_lost = False
        self.at_end = False
        self.lock.release()

    def compute_control(self):
        goal_point_map, t, self.path_index, self.is_lost, at_end = PurePursuit.pick_closest_point(
                self.pose,
                self.path,
                self.path_index,
                self.LOOK_AHEAD_DISTANCE_SQUARED)

        self.at_end = at_end or self.at_end

        # Convert the goal point to local coordinates
        goal_point_base = self.transform_point(goal_point_map)
        self.visualize(goal_point_base)

        # Get the pure pursuit angle
        angle, curvature = PurePursuit.ackermann_angle(goal_point_base, self.AXLE_LENGTH)
        angle = np.clip(angle, -self.MAX_STEERING_ANGLE, self.MAX_STEERING_ANGLE)

        # Check if we are at the end or reversing 
        if self.path_index + 2 < len(self.path) and \
                self.reverses[self.path_index + 2] == self.reverses[self.path_index + 1]:
            t = 1
        velocity = self.control_velocity(t, curvature)

        if self.reverses[self.path_index]:
            velocity *= -1

        return velocity, angle

    def control_velocity(self, t, curvature):
        velocity = self.VELOCITY_PROP/abs(curvature)
        velocity = min(velocity, self.VELOCITY)
        return max(self.MIN_VELOCITY, velocity * t)

    def drive(self, event=None):
        """
        Drive along the trajectory.
        """

        # Fetch a new pose
        self.update_pose()

        if self.path is None:
            return

        self.lock.acquire()
        # if self.is_lost or self.at_end or (self.pose is None):
        if self.pose is None:
            velocity, angle = 0., 0.
        else:
            velocity, angle = self.compute_control()

        if self.at_end:
            velocity = 0.

        # Make a message
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = self.BASE_FRAME
        drive_msg.drive.speed = velocity
        drive_msg.drive.steering_angle = angle

        # Publish it
        self.drive_pub.publish(drive_msg)
        self.lock.release()

    def update_pose(self):
        """
        Update the pose by listening
        to the transformation frame.
        """
        try:
            self.transform = self.tf_buffer.lookup_transform(self.CARTOGRAPHER_FRAME, self.BASE_FRAME, rospy.Time(0))

            self.pose = np.array([
                self.transform.transform.translation.x,
                self.transform.transform.translation.y,
                tf_conversions.transformations.euler_from_quaternion([
                    self.transform.transform.rotation.x,
                    self.transform.transform.rotation.y,
                    self.transform.transform.rotation.z,
                    self.transform.transform.rotation.w])[2]])
            self.transform = self.tf_buffer.lookup_transform(self.BASE_FRAME, self.CARTOGRAPHER_FRAME, rospy.Time(0))

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException) as e:
            pass

    def transform_point(self, point):
        """
        Convert a point to the local frame.
        """
        point_stamped = PointStamped()
        point_stamped.point.x = point[0]
        point_stamped.point.y = point[1]
        point_transformed_stamped = tf2_geometry_msgs.do_transform_point(point_stamped, self.transform)
        point_transformed = np.array([point_transformed_stamped.point.x, point_transformed_stamped.point.y])
        return point_transformed

    def visualize(self, goal_point_base):
        if self.look_ahead_pub.get_num_connections() > 0:
            points = np.array(
                    [[0., 0.],
                     [goal_point_base[0], goal_point_base[1]]])
            VisualizationUtils.plot(
                    points[:,0],
                    points[:,1],
                    self.look_ahead_pub,
                    color=(0.76, 0.335, 0.335),
                    frame=self.BASE_FRAME)


    # """
    # returns (velocity, angle)
    # """
    # def getControlAngle(self, state, visualizeMethod=None):
        # if self.pathIndex >= len(self.paths):
            # return (0,0)

        # path = self.paths[self.pathIndex]
        # points = path[0]
        # backwards = path[1]

        # goalPointGlobal, self.pointIndex, outOfBounds = PurePursuit.pickClosestPoint(
                # state.getPosition(), 
                # LOOK_AHEAD_DISTANCE_SQUARED,
                # points,
                # self.pointIndex)

        # self.outOfBounds = outOfBounds

        # # If we are at the end!
        # if np.linalg.norm(points[-1] - state.getPosition()) <= LOOK_AHEAD_DISTANCE + LOOK_AHEAD_BUFFER:
            # self.pathIndex += 1
            # self.pointIndex = 0
            # return (0,0)

        # if visualizeMethod:
            # TrajectoryTracker.visualize(state, goalPointGlobal, visualizeMethod)

        # goalPointLocal = LocalGlobalUtils.globalToLocal(state, goalPointGlobal)

        # velocity = CAR_VELOCITY
        # if len(points) - self.pointIndex < NUM_TUNE_POINTS:
            # velocity = CAR_TUNE_VELOCITY

        # if backwards:
            # velocity *= -1.

    # def isPathComplete(self):
        # return self.pathIndex >= len(self.paths)

    # @staticmethod
    # def visualize(state, goalPointGlobal, visualizeMethod):

        # points = []

        # # Visualize the goal point
        # points += [state.getPosition(), goalPointGlobal]

        # # Visualize a circle surrounding the states
        # CIRCLE_SIZE = 20
        # for i in xrange(CIRCLE_SIZE + 1):
            # angle = 2 * np.pi * i/float(CIRCLE_SIZE)
            # point = LOOK_AHEAD_DISTANCE * np.array([np.sin(angle), np.cos(angle)])
            # point += state.getPosition()
            # points.append(point)
            # if i != 0 and i != CIRCLE_SIZE:
                # points.append(point)

        # # Visualize
        # visualizeMethod(points)
