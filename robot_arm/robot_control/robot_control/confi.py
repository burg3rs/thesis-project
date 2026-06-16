
# Number of joints (excluding gripper)
NUM_JOINTS = 5

#

# Servo joint names
JOINT_NAMES = [f"joint_{i}" for i in range(NUM_JOINTS)]

# Joint limits in radians
LOWER_BOUNDS = [-1.5708, -2.65, -0.7854, -1.5708, -1.5708, 0]
UPPER_BOUNDS = [ 1.5708, 0,  1.5708,  2.26893,  1.5708, 1.5708]

#interval time
BASE_TIME = 5*1000 #time in ms

#port
DEVICE = '/dev/ttyACM1'

#joint offset scale
JOINT_OFFSET_SCALE = [-1, 1, -1, -1, -1] # [-1, 1, -1, -1, 1]

#joint offset
JOINT_OFFEST = [0, 90, 0, 0, 0] #[0, 90, 0, 0, 0]

#robot pose
HOME = [0, -120, 0, 120, 0]
