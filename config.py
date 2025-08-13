ROBOT = "g1" # Robot name, "go2", "b2", "b2w", "h1", "go2w", "g1" 
ROBOT_SCENE = "./g1/g1_29dof_with_inspire.xml" # Robot scene
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
NUM_MOTOR_HANDS = 24

START_ON_FLOOR = False
FLOOR_POSITION = [0.0, 0.0, 0.07]
FLOOR_ORIENTATION = [-0.7071, 0, 0.7071, 0.0]
FLOOR_JOINT_ANGLES = [-0.008621, -0.002926, 0.008328, 0.063897, -0.655341, 0.011785, -0.011218, -0.004299, 0.000337, 0.066435, -0.655403, -0.012579, -0.008698, -0.003402, 0.012972, 0.177344, 0.015336, -0.018787, 1.4005, -0.154848, -1.0316, 1.53425, 1.16461, -0.000033, 0.003182, 0.000881, -0.000045, -0.000014, -0.000054, -0.000019, -0.000047, -0.000015, -0.000038, -0.00001, 0.15875, -0.019087, 0.017306, 1.41357, 0.170391, -0.989743, -1.48135, 1.16461, -0.000034, 0.002237, 0.00062, -0.000044, -0.000014, -0.000052, -0.000018, -0.000046, -0.000015, -0.000037, -0.000009]

class HandJointIndex:
    LeftThumb1      = 29
    LeftThumb2      = 30
    LeftThumb3      = 31
    LeftThumb4      = 32
    LeftIndex1      = 33
    LeftIndex2      = 34
    LeftMiddle1     = 35
    LeftMiddle2     = 36
    LeftRing1       = 37
    LeftRing2       = 38
    LeftLittle1     = 39
    LeftLittle2     = 40

    RightThumb1     = 41
    RightThumb2     = 42
    RightThumb3     = 43
    RightThumb4     = 44
    RightIndex1     = 45
    RightIndex2     = 46
    RightMiddle1    = 47
    RightMiddle2    = 48
    RightRing1      = 49
    RightRing2      = 50
    RightLittle1    = 51
    RightLittle2    = 52

class G1JointIndex:
    # Left leg
    LeftHipPitch = 0
    LeftHipRoll = 1
    LeftHipYaw = 2
    LeftKnee = 3
    LeftAnklePitch = 4
    LeftAnkleB = 4
    LeftAnkleRoll = 5
    LeftAnkleA = 5

    # Right leg
    RightHipPitch = 6
    RightHipRoll = 7
    RightHipYaw = 8
    RightKnee = 9
    RightAnklePitch = 10
    RightAnkleB = 10
    RightAnkleRoll = 11
    RightAnkleA = 11

    # Waist
    WaistYaw = 12
    WaistRoll = 13        # NOTE: INVALID for g1 23dof/29dof with waist locked
    WaistA = 13           # NOTE: INVALID for g1 23dof/29dof with waist locked
    WaistPitch = 14       # NOTE: INVALID for g1 23dof/29dof with waist locked
    WaistB = 14           # NOTE: INVALID for g1 23dof/29dof with waist locked

    # Left arm
    LeftShoulderPitch = 15
    LeftShoulderRoll = 16
    LeftShoulderYaw = 17
    LeftElbow = 18
    LeftWristRoll = 19
    LeftWristPitch = 20
    LeftWristYaw = 21

    # Right arm
    RightShoulderPitch = 22
    RightShoulderRoll = 23
    RightShoulderYaw = 24
    RightElbow = 25
    RightWristRoll = 26
    RightWristPitch = 27
    RightWristYaw = 28
