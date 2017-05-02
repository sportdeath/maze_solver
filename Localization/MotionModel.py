import rospy
import numpy as np
from utils import Utils
from nav_msgs.msg import Odometry

class MotionModel:

    def __init__(self, updateModelFunction, mapBoundaries):
        self.updateModelFunction = updateModelFunction

        (self.x, self.y, self.theta) = (np.float32(0), np.float32(0), np.float32(0))

        self.mapMinX = np.float32(mapBoundaries[0])
        self.mapMinY = np.float32(mapBoundaries[1])
        self.mapMaxX = np.float32(mapBoundaries[2])
        self.mapMaxY = np.float32(mapBoundaries[3])

        self.odometrySubscriber  = \
                rospy.Subscriber(
                        rospy.get_param("~odometry_topic", "/odom"), 
                        Odometry, 
                        self.odometryCallback, 
                        queue_size=1
                        )

        # Randomness model Gaussian parameters
        # In order [x, y, theta]
        self.stdDevSlope = [np.float32(.5), np.float32(.5), np.float32(.7)] # 0.4, 0.4, 0.7
        self.stdDevOffset = [np.float32(.2), np.float32(.2), np.float32(.3)]

    # The callback function called whenever
    # we receive odometry data.
    def odometryCallback(self, msg):
        x = np.float32(msg.pose.pose.position.x)
        y = np.float32(msg.pose.pose.position.y)
        theta = np.float32(Utils.quaternion_to_angle(msg.pose.pose.orientation))

        (self.dx, self.dy, self.dTheta) = np.array([x, y, theta]) - np.array([self.x, self.y, self.theta])

        [self.x, self.y, self.theta] = [x, y, theta]

        # This topic is slower than lidar.
        # Use this to update MCL
        self.updateModelFunction()

    def updateDistribution(self, particleDistribution):

    # particleDistribution - array of size Nx3
    # action - a list of size 1x3
    # 
    # The column order of particleDistribution and
    # action is [x, y, theta]
    #
    # returns - an array of size Nx3

        # book-keeping
        columnHeight = particleDistribution.shape[0]

        # The particle frame of reference
        particleThetas = particleDistribution[:,2]
        particleThetas = particleThetas.reshape((columnHeight, 1))

        # The odometry commands are in the self.theta 
        # coordinate frame. Each particle is in its own
        # frame given by each particle theta. Therefore
        # we want to rotate each odometry command by
        odomTransformThetas = particleThetas - self.theta

        # Generate noise for dx, dy
        xNoise = np.random.normal(
                loc = np.float32(0), 
                scale = self.stdDevSlope[0] * abs(self.dx) + self.stdDevOffset[0],
                size = (columnHeight,1)
                ).astype(np.float32);
        yNoise = np.random.normal(
                loc = np.float32(0), 
                scale = self.stdDevSlope[1] * abs(self.dy) + self.stdDevOffset[1],
                size = (columnHeight,1)
                ).astype(np.float32);
                
        # Generate noise for dTheta
        # and center it around dTheta to get the
        # the theta differential vector
        thetaDifferential = np.random.normal(
                loc = self.dTheta, 
                scale = self.stdDevSlope[2] * abs(self.dTheta) + self.stdDevOffset[2],
                size = (columnHeight,1)
                ).astype(np.float32);

        # Make matrices whose rows rotate
        # the global position differential into the
        # particle frame of reference
        #
        # Ideally we want to apply noise of the form
        # (dx+epsilon) where epsilon is the noise term. 
        # To add this noise particle-wise, we
        # add the noise to the rotation component rather
        # than applying the noise and then applying the rotation.
        # This allows us to add noise and rotate with a single
        # matrix multiplication.
        # For example, in the absence of noise if we wanted
        # the rotation component to be cos(theta) we
        # compute the new rotation component as
        #
        # (cos(theta) * (1 + epsilon/dx))
        # 
        # such that
        # 
        # (cos(theta) * (1 + epsilon/dx)) * dx = 
        # cos(theta) * (dx + epsilon)

        # Prevent division by zero errors
        if self.dx != 0:
            xNoiseFactor = 1 + xNoise/self.dx
        else:
            xNoiseFactor = xNoise
            self.dx = 1
        if self.dy != 0:
            yNoiseFactor = 1 + yNoise/self.dy
        else:
            yNoiseFactor = yNoise
            self.dy = 1

        # Precompute trig
        sinOdomTransformThetas = np.sin(odomTransformThetas)
        cosOdomTransformThetas = np.cos(odomTransformThetas)

        # Create rotation matrices
        xRotation = np.concatenate((
            np.multiply( cosOdomTransformThetas, xNoiseFactor),
            np.multiply(-sinOdomTransformThetas, yNoiseFactor)
            ), axis=1)
        yRotation = np.concatenate((
            np.multiply( sinOdomTransformThetas, xNoiseFactor),
            np.multiply( cosOdomTransformThetas, yNoiseFactor)
            ), axis=1)


        # Multiply by the differential vector
        # by the rotation matrices
        dPosition = np.array([[self.dx], [self.dy]])
        # The change in x with noise
        xDifferential = np.dot(xRotation, dPosition)
        # The change in y with noise
        yDifferential = np.dot(yRotation, dPosition)

        # Concatenate the various differentials
        # into a total action differential
        action = np.concatenate((
            xDifferential, 
            yDifferential, 
            thetaDifferential
            ), axis=1)

        newParticleDistribution = particleDistribution + action

        newParticleDistribution[:,0] = np.clip(newParticleDistribution[:,0], self.mapMinX, self.mapMaxX)
        newParticleDistribution[:,1] = np.clip(newParticleDistribution[:,1], self.mapMinY, self.mapMaxY)

        newParticleDistribution = newParticleDistribution.astype(np.float32)

        # Add the action differential to the
        # existing particle distribution
        return newParticleDistribution
