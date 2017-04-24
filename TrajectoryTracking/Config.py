####################################
##        Tunable constants
## Tweak these to tune performance
####################################

# The velocity of the car
CAR_VELOCITY = 2.5 # meters/seconds

# The reaction time the car has
# to adjust to a new goal point.
# This is used to determine the
# look ahead distance in pure pursuit
REACTION_TIME = 2.0 # seconds

#####################
## Path file names
#####################

POSE_PICKER_OUTPUT = "/home/racecar/racecar-ws/src/lab6/src/TrajectoryTracking/PickedPoints.txt"
PATH_PLANNER_OUTPUT = "/home/racecar/racecar-ws/src/lab6/src/TrajectoryTracking/Path.txt"
PATH_PLANNER_OUTPUT_FIXED = "/home/racecar/racecar-ws/src/lab6/src/TrajectoryTracking/Path-Fixed.txt"
PATH_PLANNER_STATES = "/home/racecar/racecar-ws/src/lab6/src/TrajectoryTracking/PathStates.txt"
PATH_PLANNER_INDEX = "/home/racecar/racecar-ws/src/lab6/src/TrajectoryTracking/PathIndex.txt"

#################################
## Static or measured constants
##      DO NOT CHANGE
#################################

# The length of the car
#
# measured from the
# center of the front wheel axle 
# to the
# center of the back wheel axle
CAR_LENGTH = 32.5/100. # meters

# The look-ahead distance for
# choosing points to feed into
# pure pursuit
LOOK_AHEAD_DISTANCE = CAR_VELOCITY * REACTION_TIME # meters