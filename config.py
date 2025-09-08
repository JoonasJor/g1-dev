import numpy as np

ROBOT = "g1" # Robot name, "go2", "b2", "b2w", "h1", "go2w", "g1" 
ROBOT_SCENE = "./g1/scene_29dof_with_inspire.xml" # Robot scene
DOMAIN_ID = 1 # Domain id
INTERFACE = "lo" # Interface 

USE_JOYSTICK = False # Simulate Unitree WirelessController using a gamepad
JOYSTICK_TYPE = "xbox" # support "xbox" and "switch" gamepad layout
JOYSTICK_DEVICE = 0 # Joystick number

PRINT_SCENE_INFORMATION = True # Print link, joint and sensors information of robot
ENABLE_ELASTIC_BAND = True # Virtual spring band, used for lifting h1

SIMULATE_DT = 0.005  # Need to be larger than the runtime of viewer.sync()
VIEWER_DT = 0.02  # 50 fps for viewer

NUM_MOTOR_BODY = 29
NUM_MOTOR_FINGERS = 12 # Single hand

TOPIC_HAND_CMD = "rt/inspire_hand/ctrl"
TOPIC_HAND_STATE = "rt/inspire_hand/state"
TOPIC_HAND_TOUCH = "rt/inspire_hand/touch"

TOPIC_BODY_CMD = "rt/lowcmd"
TOPIC_BODY_LOW_STATE = "rt/lowstate"
TOPIC_BODY_HIGH_STATE = "rt/sportmodestate"

TOPIC_ARM_SDK = "rt/arm_sdk"
ARM_SDK_IDX = 29

TOPIC_WIRELESS_CONTROLLER = "rt/wirelesscontroller"

LOCK_LOWER_BODY = False # Lock lower body joints to default angles

START_ON_FLOOR = False
FLOOR_POSITION = [0.0, 0.0, 0.07]
FLOOR_ORIENTATION = [-0.7071, 0, 0.7071, 0.0]
FLOOR_JOINT_ANGLES = [-0.008621, -0.002926, 0.008328, 0.063897, -0.655341, 0.011785, -0.011218, -0.004299, 0.000337, 0.066435, -0.655403, -0.012579, -0.008698, -0.003402, 0.012972, 0.177344, 0.015336, -0.018787, 1.4005, -0.154848, -1.0316, 1.53425, 1.16461, -0.000033, 0.003182, 0.000881, -0.000045, -0.000014, -0.000054, -0.000019, -0.000047, -0.000015, -0.000038, -0.00001, 0.15875, -0.019087, 0.017306, 1.41357, 0.170391, -0.989743, -1.48135, 1.16461, -0.000034, 0.002237, 0.00062, -0.000044, -0.000014, -0.000052, -0.000018, -0.000046, -0.000015, -0.000037, -0.000009]
