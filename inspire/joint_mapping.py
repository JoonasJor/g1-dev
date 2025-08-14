import numpy as np
import config as cfg
import inspire.inspire_dds as inspire_dds
import traceback

# fix this abomination later
JOINT_MAPPING_R = np.array([
    [cfg.HandJointIndex_R.RightLittle1, cfg.HandJointIndex_R.RightLittle2], # Pinky
    [cfg.HandJointIndex_R.RightRing1,   cfg.HandJointIndex_R.RightRing2],   # Ring
    [cfg.HandJointIndex_R.RightMiddle1, cfg.HandJointIndex_R.RightMiddle2], # Middle
    [cfg.HandJointIndex_R.RightIndex1,  cfg.HandJointIndex_R.RightIndex2],  # Index
    [cfg.HandJointIndex_R.RightThumb3,  cfg.HandJointIndex_R.RightThumb4],  # Thumb-bend
    [cfg.HandJointIndex_R.RightThumb1,  cfg.HandJointIndex_R.RightThumb2]   # Thumb-rotation
]) - cfg.NUM_MOTOR_BODY - 12

JOINT_MAPPING_L = np.array([
    [cfg.HandJointIndex_L.LeftLittle1,  cfg.HandJointIndex_L.LeftLittle2],  # Pinky
    [cfg.HandJointIndex_L.LeftRing1,    cfg.HandJointIndex_L.LeftRing2],    # Ring
    [cfg.HandJointIndex_L.LeftMiddle1,  cfg.HandJointIndex_L.LeftMiddle2],  # Middle
    [cfg.HandJointIndex_L.LeftIndex1,   cfg.HandJointIndex_L.LeftIndex2],   # Index
    [cfg.HandJointIndex_L.LeftThumb3,   cfg.HandJointIndex_L.LeftThumb4],   # Thumb-bend
    [cfg.HandJointIndex_L.LeftThumb1,   cfg.HandJointIndex_L.LeftThumb2]    # Thumb-rotation
])- cfg.NUM_MOTOR_BODY

JOINT_NAMES_R = [
    "right_little_1_joint", "right_little_2_joint",
    "right_ring_1_joint", "right_ring_2_joint",
    "right_middle_1_joint", "right_middle_2_joint",
    "right_index_1_joint", "right_index_2_joint",
    "right_thumb_1_joint", "right_thumb_2_joint",
    "right_thumb_3_joint", "right_thumb_4_joint"
]

JOINT_NAMES_L = [
    "left_little_1_joint", "left_little_2_joint",
    "left_ring_1_joint", "left_ring_2_joint",
    "left_middle_1_joint", "left_middle_2_joint",
    "left_index_1_joint", "left_index_2_joint",
    "left_thumb_1_joint", "left_thumb_2_joint",
    "left_thumb_3_joint", "left_thumb_4_joint"
]

def get_joint_limits(mj_model, l_r):
    """
    Returns (angle_limits, force_limits) for the hand joints in the correct order.
    """
    if l_r == "r":
        joint_indices = cfg.HandJointIndex_R.idx_list()
    else:
        joint_indices = cfg.HandJointIndex_L.idx_list()

    angle_limits = []
    force_limits = []

    for idx in joint_indices:
        angle_limits.append(tuple(mj_model.jnt_range[idx]))
        force_limits.append(tuple(mj_model.actuator_forcerange[idx]))

    return angle_limits, force_limits

def dds_to_mujoco(angles_6, forces_6, speeds_6, l_r="r"):
    """
    FOR TESTING. Figure out how to do this properly later.

    Expand 6 joints to 12 joints.
    """

    if l_r == "r":
        mapping = JOINT_MAPPING_R
    else:
        mapping = JOINT_MAPPING_L

    angles_12 = [0.0] * 12
    forces_12 = [0.0] * 12
    speeds_12 = [0.0] * 12

    for i, joint_group in enumerate(mapping):
        angles_12[joint_group[0]] = angles_6[i] / 2.0
        angles_12[joint_group[1]] = angles_6[i] / 2.0

        forces_12[joint_group[0]] = forces_6[i] / 2.0
        forces_12[joint_group[1]] = forces_6[i] / 2.0

        speeds_12[joint_group[0]] = speeds_6[i] / 2.0
        speeds_12[joint_group[1]] = speeds_6[i] / 2.0

    return angles_12, forces_12, speeds_12

def mujoco_to_dds(angles_12, forces_12, l_r="r"):
    """
    FOR TESTING. Figure out how to do this properly later.

    Compress 12 joints to 6 joints.
    """

    if l_r == "r":
        mapping = JOINT_MAPPING_R
    else:
        mapping = JOINT_MAPPING_L

    angles_6 = [0] * 6
    forces_6 = [0] * 6

    try:
        for i, joint_group in enumerate(mapping):
            angles_6[i] = angles_12[joint_group[0]] + angles_12[joint_group[1]]
            forces_6[i] = forces_12[joint_group[0]] + forces_12[joint_group[1]]
    except IndexError as e:
        traceback.print_exc()
        print(f"[mujoco_to_dds_{l_r}] error: {e} - {i=}, {joint_group=}")
        print(f"{len(angles_6)=}")
        print(f"{len(angles_12)=}")

    return angles_6, forces_6