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


class ParticleFilter():
    def __init__(self):
        # parameters
        self.MAX_PARTICLES = int(rospy.get_param("~max_particles"))
        self.MAX_VIZ_PARTICLES = int(rospy.get_param("~max_viz_particles"))
        self.MAX_RANGE_METERS = float(rospy.get_param("~max_range"))
        self.MAX_RANGE_PX = None

        # cddt and glt range methods are discrete, this defines number of discrete thetas
        self.THETA_DISCRETIZATION = int(rospy.get_param("~theta_discretization"))
        self.WHICH_RANGE_METHOD = rospy.get_param("~range_method", "cddt")

        # various data containers used in the MCL algorithm
        self.map_info = None
        self.map_initialized = False
        self.range_method = None

        # use this lock for controlling accesses to the particles
        # necessary for avoiding concurrency errors
        self.state_lock = Lock()

        # initialize the state
        self.get_omap()

        # particle poses and weights - particles should be N by 3
        self.initializeParticles()

        # uniform prior
        self.weights = np.ones(self.MAX_PARTICLES) / float(self.MAX_PARTICLES)

        # these topics are for visualization, feel free to add, remove, or change as you see fit
        self.pose_pub      = rospy.Publisher("/pf/viz/inferred_pose", PoseStamped, queue_size = 1)
        self.particle_pub  = rospy.Publisher("/pf/viz/particles", PoseArray, queue_size = 1)
        self.pub_fake_scan = rospy.Publisher("/pf/viz/fake_scan", LaserScan, queue_size = 1)
        
        # use this for your inferred transformations
        self.pub_tf = tf.TransformBroadcaster()

        # these integrate with RViz
        self.pose_sub  = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.clicked_pose, queue_size=1)
        self.click_sub = rospy.Subscriber("/clicked_point", PointStamped, self.clicked_pose, queue_size=1)

        self.motionModel = None
        self.initializeMotionModel()

        self.sensorModel = SensorModel(self.range_method, self.MAX_RANGE_PX)

        rospy.loginfo("Finished initializing, waiting on messages...")

    def initializeMotionModel(self):
        mapWidth = self.map_info.width
        mapHeight = self.map_info.height

        THRESHOLD = 4
        mapMax = np.array([THRESHOLD,THRESHOLD,0])
        mapMin = np.array([mapWidth - 1 - THRESHOLD, mapHeight - 1 - THRESHOLD, 0])

        mapMin = Utils.map_to_world_slow(mapMin[0], mapMin[1], mapMin[2], self.map_info)
		mapMax = Utils.map_to_world_slow(mapMax[0], mapMax[1], mapMax[2], self.map_info)

        self.motionModel = MotionModel(self.update, [mapMin[0], mapMin[1], mapMax[0], mapMax[1]])

    def initializeParticles(self):
        mapWidth = self.map_info.width
        mapHeight = self.map_info.height

        self.particles = np.zeros((self.MAX_PARTICLES, 3),dtype=np.float32)

        # for each particle
        for particleIndex in xrange(self.MAX_PARTICLES):
            while True:
                # Choose random point in map
                particleColumn = np.random.randint(low=0,high=mapWidth)
                particleRow = np.random.randint(low=0,high=mapHeight)

                # check if point is occupied
                dataPoint = particleRow * mapWidth + particleColumn

                # if it is not occupied break
                if self.map_msg.data[dataPoint] == 0:
                    particleTheta = np.random.uniform(low=-np.pi, high=np.pi)
                    mapCellArray = np.array([particleColumn, particleRow, particleTheta])
                    self.particles[particleIndex,:] = Utils.map_to_world_slow(mapCellArray, self.map_info)
                    break

    def get_omap(self):
        # this way you could give it a different map server as a parameter
        map_service_name = rospy.get_param("~static_map", "static_map")
        rospy.loginfo("getting map from service: %s", map_service_name)
        rospy.wait_for_service(map_service_name)
        map_msg = rospy.ServiceProxy(map_service_name, GetMap)().map
        self.map_msg = map_msg

        self.map_info = map_msg.info
        oMap = range_libc.PyOMap(map_msg)
        # this value is the max range used internally in RangeLibc
        # it also should be the size of your sensor model table
        self.MAX_RANGE_PX = int(self.MAX_RANGE_METERS / self.map_info.resolution)

        # initialize range method
        rospy.loginfo("Initializing range method: %s", self.WHICH_RANGE_METHOD)
        if self.WHICH_RANGE_METHOD == "bl":
            self.range_method = range_libc.PyBresenhamsLine(oMap, self.MAX_RANGE_PX)
        elif "cddt" in self.WHICH_RANGE_METHOD:
            self.range_method = range_libc.PyCDDTCast(oMap, self.MAX_RANGE_PX, self.THETA_DISCRETIZATION)
            if self.WHICH_RANGE_METHOD == "pcddt":
                rospy.loginfo("Pruning...")
                self.range_method.prune()
        elif self.WHICH_RANGE_METHOD == "rm":
            self.range_method = range_libc.PyRayMarching(oMap, self.MAX_RANGE_PX)
        elif self.WHICH_RANGE_METHOD == "rmgpu":
            self.range_method = range_libc.PyRayMarchingGPU(oMap, self.MAX_RANGE_PX)
        elif self.WHICH_RANGE_METHOD == "glt":
            self.range_method = range_libc.PyGiantLUTCast(oMap, self.MAX_RANGE_PX, self.THETA_DISCRETIZATION)
        rospy.loginfo("Done loading map")

         # 0: permissible, -1: unmapped, large value: blocked
        array_255 = np.array(map_msg.data).reshape((map_msg.info.height, map_msg.info.width))

        # 0: not permissible, 1: permissible
        # this may be useful for global particle initialization - don't initialize particles in non-permissible states
        self.permissible_region = np.zeros_like(array_255, dtype=bool)
        self.permissible_region[array_255==0] = 1
        self.map_initialized = True

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

    # returns the expected value of the pose given the particle distribution
    def expected_pose(self):
        return np.average(self.particles, axis=0, weights=self.weights)


    def update(self):
        if sensor_model.lidar_initialized() and self.map_initialized:
            if self.state_lock.locked():
                rospy.loginfo("Concurrency error avoided")
            else:
                self.state_lock.acquire()
                
                self.particle_indices = np.random.choice(np.arange(self.MAX_PARTICLES),self.MAX_PARTICLES,replace=True,p=self.weights)
                self.particles = self.particles[self.particle_indices, :]
                
                # Update with motion model
                self.particles = self.motionModel.updateDistribution(self.particles)

                self.sensorModel.updateSensorModel(self.particles, self.weights)    # observation is the lidar data
                
                self.inferred_pose = self.expected_pose()
                
                self.state_lock.release()
                self.visualize()


if __name__=="__main__":
    rospy.init_node("particle_filter")
    pf = ParticleFilter()
    rospy.spin()