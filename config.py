import numpy as np

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

START_ON_FLOOR = True
FLOOR_POSITION = [0.0, 0.0, 0.07]
FLOOR_ORIENTATION = [-0.7071, 0, 0.7071, 0.0]
FLOOR_JOINT_ANGLES = [-0.008621, -0.002926, 0.008328, 0.063897, -0.655341, 0.011785, -0.011218, -0.004299, 0.000337, 0.066435, -0.655403, -0.012579, -0.008698, -0.003402, 0.012972, 0.177344, 0.015336, -0.018787, 1.4005, -0.154848, -1.0316, 1.53425, 1.16461, -0.000033, 0.003182, 0.000881, -0.000045, -0.000014, -0.000054, -0.000019, -0.000047, -0.000015, -0.000038, -0.00001, 0.15875, -0.019087, 0.017306, 1.41357, 0.170391, -0.989743, -1.48135, 1.16461, -0.000034, 0.002237, 0.00062, -0.000044, -0.000014, -0.000052, -0.000018, -0.000046, -0.000015, -0.000037, -0.000009]

class JointDefaults:
    # -- G1 body joint parameters --
    angles_body = np.array([
        # Left leg
        -0.1, 0.0, 0.0, 0.3, -0.2, 0.0,
        # Right leg
        -0.1, 0.0, 0.0, 0.3, -0.2, 0.0,
        # Waist
        0.0, 0.0, 0.0,
        # Left arm
        0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 0.0,
        # Right arm
        0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 0.0
    ])

    kp_body = np.array([
        # Left leg
        60, 60, 60, 100, 40, 40,
        # Right leg
        60, 60, 60, 100, 40, 40,
        # Waist
        60, 40, 40,
        # Left arm
        40, 40, 40, 40, 40, 40, 40,
        # Right arm
        40, 40, 40, 40, 40, 40, 40
    ])

    kd_body = np.array([
        # Left leg
        1, 1, 1, 2, 1, 1,
        # Right leg
        1, 1, 1, 2, 1, 1,
        # Waist
        1, 1, 1,
        # Left arm
        1, 1, 1, 1, 1, 1, 1,
        # Right arm
        1, 1, 1, 1, 1, 1, 1
    ])

    angle_limits_body = [
        (-2.5307, 2.8798),   # LeftHipPitch
        (-0.5236, 2.9671),   # LeftHipRoll
        (-2.7576, 2.7576),   # LeftHipYaw
        (-0.087267, 2.8798), # LeftKnee
        (-0.87267, 0.5236),  # LeftAnklePitch
        (-0.2618, 0.2618),   # LeftAnkleRoll
        (-2.5307, 2.8798),   # RightHipPitch
        (-2.9671, 0.5236),   # RightHipRoll
        (-2.7576, 2.7576),   # RightHipYaw
        (-0.087267, 2.8798), # RightKnee
        (-0.87267, 0.5236),  # RightAnklePitch
        (-0.2618, 0.2618),   # RightAnkleRoll
        (-2.618, 2.618),     # WaistYaw
        (-0.52, 0.52),       # WaistRoll
        (-0.52, 0.52),       # WaistPitch
        (-3.0892, 2.6704),   # LeftShoulderPitch
        (-1.5882, 2.2515),   # LeftShoulderRoll
        (-2.618, 2.618),     # LeftShoulderYaw
        (-1.0472, 2.0944),   # LeftElbow
        (-1.9722, 1.9722),   # LeftWristRoll
        (-1.6144, 1.6144),   # LeftWristPitch
        (-1.6144, 1.6144),   # LeftWristYaw
        (-3.0892, 2.6704),   # RightShoulderPitch
        (-2.2515, 1.5882),   # RightShoulderRoll
        (-2.618, 2.618),     # RightShoulderYaw
        (-1.0472, 2.0944),   # RightElbow
        (-1.9722, 1.9722),   # RightWristRoll
        (-1.6144, 1.6144),   # RightWristPitch
        (-1.6144, 1.6144)    # RightWristYaw
    ]

    # -- Inspire hand joint parameters --
    angles_hand = np.array([
        # Left fingers
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        # Right fingers
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    ])

    kp_hand = np.array([
        # Left fingers
        20, 20, 20, 20, 20, 20,
        20, 20, 20, 20, 20, 20,
        # Right fingers
        20, 20, 20, 20, 20, 20,
        20, 20, 20, 20, 20, 20
    ])

    kd_hand = np.array([
        # Left fingers
        1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1,
        # Right fingers
        1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1
    ])

    # Combined parameters
    angles = np.concatenate([angles_body, angles_hand])
    kp = np.concatenate([kp_body, kp_hand])
    kd = np.concatenate([kd_body, kd_hand])

from enum import IntEnum

class HandJointIndex_L(IntEnum):
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

    @classmethod
    def idx_list(cls):
        return [member.value for member in cls]


class HandJointIndex_R(IntEnum):
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

    @classmethod
    def idx_list(cls):
        return [member.value for member in cls]

class G1JointIndex(IntEnum):
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

    @classmethod
    def idx_list(cls):
        return [member.value for member in cls]
