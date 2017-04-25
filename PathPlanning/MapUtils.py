import numpy as np

import rospy

import range_libc
from nav_msgs.srv import GetMap
import tf

class MapUtils:

    @staticmethod
    def getMap():
        # Get the map
        map_service_name = rospy.get_param("~static_map", "static_map")
        rospy.loginfo("Getting map from service: %s", map_service_name)
        rospy.wait_for_service(map_service_name)
        rospy.loginfo("Got map!")

        # The map message
        return rospy.ServiceProxy(map_service_name, GetMap)().map

    @staticmethod
    def getRangeLib(mapMsg):
        # Pre-compute range lib
        oMap = range_libc.PyOMap(mapMsg)
        MAX_RANGE_METERS = 20
        MAX_RANGE_PX = int(MAX_RANGE_METERS / mapMsg.info.resolution)
        THETA_DISCRETIZATION = 200
        rangeLib = range_libc.PyCDDTCast(oMap, MAX_RANGE_PX, THETA_DISCRETIZATION)
        rangeLib.prune()
        rospy.loginfo("range_libc initialized")
        return rangeLib

    @staticmethod
    def getUnoccupiedPoints(mapMsg):
        # Pre-compute the unoccupied points in the
        # map for sampling
        unoccupiedPoints = []
        for i in xrange(mapMsg.info.height):
            for j in xrange(mapMsg.info.width):
                index = i * mapMsg.info.width + j
                occupied = mapMsg.data[index]
                if occupied == 0:
                    unoccupiedPoints.append(MapUtils.mapCellToMeters((i,j),mapMsg))
        return unoccupiedPoints

    @staticmethod
    def quaternionToAngle(q):
        """Convert a quaternion _message_ into an angle in radians.
        The angle represents the yaw.
        This is not just the z component of the quaternion."""
        x, y, z, w = q.x, q.y, q.z, q.w
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((x, y, z, w))
        return yaw

    @staticmethod
    def mapCellToMeters(mapCell, mapMsg):
        y = -float(mapCell[0]) * mapMsg.info.resolution + mapMsg.info.origin.position.y
        x = -float(mapCell[1]) * mapMsg.info.resolution + mapMsg.info.origin.position.x
        return np.array([x,y])

    @staticmethod
    def metersToMapCell(meters, mapMsg):
        y = (-meters[0] + mapMsg.info.origin.position.x)/mapMsg.info.resolution
        x = (-meters[1] + mapMsg.info.origin.position.y)/mapMsg.info.resolution
        return np.array([int(x),int(y)])

    @staticmethod
    def getRangeMethod(rangeLib, mapMsg):

        def rangeMethod(meters, angle):
            # Convert to pixels
            mapCell = MapUtils.metersToMapCell(meters, mapMsg)

            distanceToObstacle = rangeLib.calc_range(
                    mapCell[0],
                    mapCell[1],
                    angle + np.pi)

            # Convert back to meters
            distanceToObstacle *= mapMsg.info.resolution

            return distanceToObstacle

        return rangeMethod
