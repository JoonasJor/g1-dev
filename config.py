ROBOT = "g1" # Robot name, "go2", "b2", "b2w", "h1", "go2w", "g1" 
ROBOT_SCENE = "./g1/g1_29dof_with_inspire.xml" # Robot scene
DOMAIN_ID = 1 # Domain id
INTERFACE = "lo" # Interface 

USE_JOYSTICK = False # Simulate Unitree WirelessController using a gamepad
JOYSTICK_TYPE = "xbox" # support "xbox" and "switch" gamepad layout
JOYSTICK_DEVICE = 0 # Joystick number

PRINT_SCENE_INFORMATION = True # Print link, joint and sensors information of robot
ENABLE_ELASTIC_BAND = False # Virtual spring band, used for lifting h1

SIMULATE_DT = 0.005  # Need to be larger than the runtime of viewer.sync()
VIEWER_DT = 0.02  # 50 fps for viewer

NUM_MOTOR_BODY = 29
NUM_MOTOR_HANDS = 24

START_ON_FLOOR = True
FLOOR_POSITION = [0.0, 0.0, 0.07]
FLOOR_ORIENTATION = [-0.7071, 0, 0.7071, 0.0]
FLOOR_JOINT_ANGLES = [-0.008621, -0.002926, 0.008328, 0.063897, -0.655341, 0.011785, -0.011218, -0.004299, 0.000337, 0.066435, -0.655403, -0.012579, -0.008698, -0.003402, 0.012972, 0.177344, 0.015336, -0.018787, 1.4005, -0.154848, -1.0316, 1.53425, 1.16461, -0.000033, 0.003182, 0.000881, -0.000045, -0.000014, -0.000054, -0.000019, -0.000047, -0.000015, -0.000038, -0.00001, 0.15875, -0.019087, 0.017306, 1.41357, 0.170391, -0.989743, -1.48135, 1.16461, -0.000034, 0.002237, 0.00062, -0.000044, -0.000014, -0.000052, -0.000018, -0.000046, -0.000015, -0.000037, -0.000009]

"""G1_JOINT_IDX = [ 
    0,  1,  2,  3,  4,  5,        # left leg
    6,  7,  8,  9,  10, 11,       # right leg
    12, 13, 14,                   # waist
    15, 16, 17, 18, 19, 20, 21,   # left arm
    34, 35, 36, 37, 38, 39, 40,   # right arm
]

INSPIRE_HAND_JOINT_IDX = [
    22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, # left fingers
    41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52  # right fingers
]"""
