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

    def __init__(self):
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
        self.is_lost = True
        self.at_end = False

        # Create a drive timer
        rospy.Timer(rospy.Duration(self.TRAJECTORY_RATE), self.drive)

    def stop(self):
        """
        Stop driving.
        """
        self.is_lost = True
        self.drive()

    def track(self, path):
        # Sample the path
        points = []
        for steer in path:
            points.append(steer.sample(self.TRAJECTORY_SAMPLE_WIDTH))

        self.path_index = 0
        self.path = np.concatenate(points, axis=0)[:, :2]
        self.is_lost = False
        self.at_end = False

    def compute_control(self):
        goal_point_map, self.path_index, self.is_lost, self.at_end = PurePursuit.pick_closest_point(
                self.pose,
                self.path,
                self.path_index,
                self.LOOK_AHEAD_DISTANCE_SQUARED)

        # If we are too far from the path
        if self.is_lost or self.at_end:
            return 0, 0

        # Convert the goal point to local coordinates
        goal_point_base = self.transform_point(goal_point_map)
        self.visualize(goal_point_base)

        angle = PurePursuit.ackermann_angle(goal_point_base, self.AXLE_LENGTH)

        # angle = np.clip(angle, -self.MAX_STEERING_ANGLE, self.MAX_STEERING_ANGLE)

        return self.VELOCITY, angle

    def drive(self, event=None):
        """
        Drive along the trajectory.
        """

        # Fetch a new pose
        self.update_pose()

        if self.is_lost or self.at_end or (self.pose is None):
            velocity, angle = 0., 0.
        else:
            velocity, angle = self.compute_control()

        # Make a message
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = self.BASE_FRAME
        drive_msg.drive.speed = velocity
        drive_msg.drive.steering_angle = angle

        # Publish it
        self.drive_pub.publish(drive_msg)

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
