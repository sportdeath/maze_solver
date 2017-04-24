#!/usr/bin/env python

'''
Lab 5 Starter Code

- Outlines a basic implementation of particle filter localization algorithm
- Initializes RangeLibc
- Uses ROS params for easier configuration
- Only sends visualization if someone is listening
- Uses locks to avoid concurreny errors
- Includes code for visualizing discretized sensor models with matplotlib
- Includes helper functions
    - Timer
    - CircusularArray
    - Utils
        - coordinate space conversions
        - useful ROS object construction functions

While developing, be careful of:
    - coordinate conversiokns
    - vectorization with numpy
    - caching and reusing large buffers
    - concurrency problems
    - implement visualization early to help debugging

To launch:

    first start the map server: 
    $ roslaunch lab5 map_server.launch
    then launch this code with:
    $ roslaunch lab5 localize.launch

Written by Corey Walsh for Spring 2017 6.141 Lab 5

'''

import rospy
import numpy as np

from timer import Timer
from circular_array import CircularArray
from utils import Utils

from std_msgs.msg import String, Header, Float32MultiArray
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseArray, Quaternion, PolygonStamped,Polygon, Point32, PoseWithCovarianceStamped, PointStamped
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetMap
import tf.transformations
import tf
import matplotlib.pyplot as plt
import range_libc
import time

from threading import Lock

from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter

from MotionModel import MotionModel


class SensorModel():
    def __init__(self, range_method):
        self.lidar_initialized = False
        self.laser_angles = None

        self.first_sensor_update = True

        # parameters
        self.MAX_PARTICLES = int(rospy.get_param("~max_particles"))
        self.MAX_VIZ_PARTICLES = int(rospy.get_param("~max_viz_particles"))
        self.MAX_RANGE_METERS = float(rospy.get_param("~max_range"))
        self.MAX_RANGE_PX = None

        # data container used in the MCL algorithm
        self.range_method = range_method

        # when possible, use these variables to cache large arrays and only make them once
        self.queries = None     
        self.ranges = None
        self.sensor_model_table = None

        # initialize the state
        self.METERS_PER_PIXEL = self.MAX_RANGE_METERS / float(self.MAX_RANGE_PX)

        # # particle poses and weights - particles should be N by 3
        # self.initializeParticles()

        # uniform prior
        self.weights = np.ones(self.MAX_PARTICLES) / float(self.MAX_PARTICLES)
        # updated weights from sensor model
        # init as array size of weights
        self.obs_update = np.ones(self.MAX_PARTICLES)/float(self.MAX_PARTICLES)

        self.sensor_model_table = self.precompute_sensor_model()
        # upload the sensor model to RangeLib for ultra fast resolution later
        self.range_method.set_sensor_model(self.sensor_model_table)

        # these topics are for visualization, feel free to add, remove, or change as you see fit
        self.pub_fake_scan = rospy.Publisher("/pf/viz/fake_scan", LaserScan, queue_size = 1)
        
        # use this for your inferred transformations
        self.pub_tf = tf.TransformBroadcaster()

        # these topics are to receive data from the racecar
        self.laser_sub = rospy.Subscriber(rospy.get_param("~scan_topic", "/scan"), LaserScan, self.lidarCB, queue_size=1)

    def precompute_sensor_model(self):
        TableType = np.float64
        GROUND_TRUTH_STD_DEV = TableType(10)
        SHORT_MEASUREMENT_OFFSET = TableType(0.025)
        SPIKE_HEIGHT = TableType(0.07)
        RANDOM_MEASUREMENT_OFFSET = TableType(0.007)
        tableWidth = int(self.MAX_RANGE_PX) + 1

        # 1/(sigma * sqrt(2*pi))
        GAUSSIAN_CONSTANT = 1./(GROUND_TRUTH_STD_DEV * np.sqrt(2*np.pi))

        sensorModelTable = np.zeros((tableWidth, tableWidth), dtype=TableType)

        # Populate the table
        for groundTruthIndex in xrange(tableWidth):
            for measuredIndex in xrange(tableWidth):
                # Ground truth probability
                # A Gaussian centered around ground truth
                value = GAUSSIAN_CONSTANT * \
                        np.exp(-np.power(measuredIndex - groundTruthIndex,2) \
                        /(2 * np.power(GROUND_TRUTH_STD_DEV, 2)))

                # Short measurement probability
                # A line that is zero at ground Truth
                # and increases as the measurement increases
                if measuredIndex < groundTruthIndex:
                    distanceToTruth = groundTruthIndex - measuredIndex
                    value += SHORT_MEASUREMENT_OFFSET * \
                           distanceToTruth/groundTruthIndex

                # Missed measurement probability
                # A spike at SPIKE_THRESHOLD with 
                if tableWidth - 1 ==  measuredIndex:
                    value += SPIKE_HEIGHT

                # Random measurement probability
                value += RANDOM_MEASUREMENT_OFFSET

                # Put the value in the table
                sensorModelTable[measuredIndex, groundTruthIndex] = TableType(value)

        # Normalize each column (each ground truth value)
        sensorModelTable = \
                sensorModelTable/ \
                np.sum(sensorModelTable, axis=0)

        return sensorModelTable

    def visualize_sensor_model_table(self):
        # code to generate visualizations of the sensor model
        if True:
            # visualize the sensor model
            fig = plt.figure()
            ax = fig.gca(projection='3d')
            table_width = np.shape(self.sensor_model_table)[0]

            # Make data.
            X = np.arange(0, table_width, 1.0)
            Y = np.arange(0, table_width, 1.0)
            X, Y = np.meshgrid(X, Y)

            # Plot the surface.
            surf = ax.plot_surface(X, Y, self.sensor_model_table, cmap="bone", rstride=2, cstride=2,
                                   linewidth=0, antialiased=True)

            ax.text2D(0.05, 0.95, "Precomputed Sensor Model", transform=ax.transAxes)
            ax.set_xlabel('Ground truth distance (in px)')
            ax.set_ylabel('Measured Distance (in px)')
            ax.set_zlabel('P(Measured Distance | Ground Truth)')

            plt.show()
        elif False:
            plt.imshow(self.sensor_model_table * 255, cmap="gray")
            plt.show()
        elif False:
            plt.plot(self.sensor_model_table[:,140])
            plt.plot([139,139],[0.0,0.08], label="test")
            plt.ylim(0.0, 0.08)
            plt.xlabel("Measured Distance (in px)")
            plt.ylabel("P(Measured Distance | Ground Truth Distance = 140px)")
            plt.show()

    def sensor_model(self):

        # only allocate buffers once to avoid slowness
        if self.first_sensor_update:
            # Initialize and store any large reused arrays here
            self.angles = np.arange(self.ANGLE_MIN, self.ANGLE_MAX, 10*self.ANGLE_INCREMENT, dtype=np.float32)
            self.ranges = np.zeros(self.angles.shape[0]*self.MAX_PARTICLES, dtype=np.float32)
            self.first_sensor_update = False

        obs = np.float32(self.scan_data[::10])

        self.range_method.calc_range_repeat_angles(self.particles, self.angles, self.ranges)
        self.range_method.eval_sensor_model(obs, self.ranges, self.weights, self.angles.shape[0], self.MAX_PARTICLES)
        
        # for calc_range_repeat_angles all ranges for a single particle are next to each other, so the quickly changing axis is angles
        # if you have 6 particles, each of which does 3 ray casts (obviously you will have more) it would be like this:
        # ranges: [p1a1,p1a2,p1a3,p2a1,p2a2,p2a3,p3a1,p3a2,p3a3,p4a1,p4a2,p4a3,p5a1,p5a2,p5a3,p6a1,p6a2,p6a3]
        
        # input to ranges and queries must be continous float32 numpy arrays

        self.weights = self.weights/np.sum(self.weights)


    def publish_tf(self, stamp=None):
        """ Publish a tf from map to base_link. """
        if stamp == None:
            stamp = rospy.Time.now()
        x = self.inferred_pose[0]
        y = self.inferred_pose[1]
        theta = self.inferred_pose[2]

        position = (x, y, 0)
        orientation = tf.transformations.quaternion_from_euler(0, 0, theta)

        self.pub_tf.sendTransform(position, orientation, stamp, "/base_link", "/map");

    def visualize(self):
        self.publish_tf()
        if self.pose_pub.get_num_connections() > 0 and isinstance(self.inferred_pose, np.ndarray):
            x = self.inferred_pose[0]
            y = self.inferred_pose[1]
            theta = self.inferred_pose[2]

            ps = PoseStamped()
            ps.header = Utils.make_header("map")
            # FILL OUT THE POSE 
            # Utils.angle_to_quaternion() will probably be useful
            ps.pose = Pose(Point(x, y, 0), Utils.angle_to_quaternion(theta))
            self.pose_pub.publish(ps)

        if self.particle_pub.get_num_connections() > 0:
            if self.MAX_PARTICLES > self.MAX_VIZ_PARTICLES:
                # randomly downsample particles to avoid killing RViz with tons of particles
                proposal_indices = np.random.choice(self.particle_indices, self.MAX_VIZ_PARTICLES, p=self.weights)
                self.publish_particles(self.particles[proposal_indices,:])
            else:
                self.publish_particles(self.particles)
        if self.pub_fake_scan.get_num_connections() > 0 and isinstance(self.ranges, np.ndarray):
            # generate the scan from the point of view of the inferred position for visualization
            # this should always align with the map, if not then you are probably using RangeLibc wrong 
            # or you have a coordinate space issue
            rospy.loginfo("FAKE NEWS")
            self.viz_queries[:,0] = self.inferred_pose[0]
            self.viz_queries[:,1] = self.inferred_pose[1]
            self.viz_queries[:,2] = self.downsampled_angles + self.inferred_pose[2]
            self.range_method.calc_range_many(self.viz_queries, self.viz_ranges)
            self.publish_scan(self.downsampled_angles, self.viz_ranges)

    def publish_particles(self, particles):
        pa = PoseArray()
        pa.header = Utils.make_header("map")
        pa.poses = Utils.particles_to_poses(particles)

        self.particle_pub.publish(pa)

    def publish_scan(self, angles, ranges):
        ls = LaserScan()
        ls.header = Utils.make_header("laser", stamp=self.last_stamp)
        ls.angle_min = np.min(angles)
        ls.angle_max = np.max(angles)
        ls.angle_increment = np.abs(angles[0] - angles[1])
        ls.range_min = 0
        ls.range_max = np.max(ranges)
        ls.ranges = ranges
        self.pub_fake_scan.publish(ls)

    def lidarCB(self, msg):
        if not isinstance(self.laser_angles, np.ndarray):
            rospy.loginfo("...Received first LiDAR message")
            
            self.laser_angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
            self.viz_queries = None
            self.scan_data = None
            self.ANGLE_MIN = msg.angle_min
            self.ANGLE_MAX = msg.angle_max
            self.ANGLE_INCREMENT = msg.angle_increment

        # store anything used in MCL update
        self.scan_data = np.minimum(msg.ranges, msg.range_max)
        rospy.loginfo("scan data:")
        rospy.loginfo(self.scan_data)
        self.lidar_initialized = True


if __name__=="__main__":
    rospy.init_node("particle_filter")
    pf = ParticleFilter()
    rospy.spin()
