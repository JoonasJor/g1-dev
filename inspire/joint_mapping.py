import g1_joints as Joints

# This maps 12 physical joints to 6 Inspire joints for DDS communication
# TODO: Check correct mapping for Thumb-bend and Thumb-rotation
JOINT_MAPPING_R = [
    [Joints.Fingers_R.RightLittle1.motor_idx, Joints.Fingers_R.RightLittle2.motor_idx], # Pinky
    [Joints.Fingers_R.RightRing1.motor_idx,   Joints.Fingers_R.RightRing2.motor_idx],   # Ring
    [Joints.Fingers_R.RightMiddle1.motor_idx, Joints.Fingers_R.RightMiddle2.motor_idx], # Middle
    [Joints.Fingers_R.RightIndex1.motor_idx,  Joints.Fingers_R.RightIndex2.motor_idx],  # Index
    [Joints.Fingers_R.RightThumb3.motor_idx,  Joints.Fingers_R.RightThumb4.motor_idx],  # Thumb-bend
    [Joints.Fingers_R.RightThumb1.motor_idx,  Joints.Fingers_R.RightThumb2.motor_idx]   # Thumb-rotation
]

JOINT_MAPPING_L = [
    [Joints.Fingers_L.LeftLittle1.motor_idx,  Joints.Fingers_L.LeftLittle2.motor_idx],  # Pinky
    [Joints.Fingers_L.LeftRing1.motor_idx,    Joints.Fingers_L.LeftRing2.motor_idx],    # Ring
    [Joints.Fingers_L.LeftMiddle1.motor_idx,  Joints.Fingers_L.LeftMiddle2.motor_idx],  # Middle
    [Joints.Fingers_L.LeftIndex1.motor_idx,   Joints.Fingers_L.LeftIndex2.motor_idx],   # Index
    [Joints.Fingers_L.LeftThumb3.motor_idx,   Joints.Fingers_L.LeftThumb4.motor_idx],   # Thumb-bend
    [Joints.Fingers_L.LeftThumb1.motor_idx,   Joints.Fingers_L.LeftThumb2.motor_idx]    # Thumb-rotation
]

def expand(angles_6, forces_6, speeds_6, l_r="r"):
    """
    Expand 6 joints to 12 joints.
    """

    if l_r == "r":
        joint_mapping = JOINT_MAPPING_R
    else:
        joint_mapping = JOINT_MAPPING_L

    angles_12 = [0.0] * 12
    forces_12 = [0.0] * 12
    speeds_12 = [0.0] * 12

    for i, (j1, j2) in enumerate(joint_mapping):
        angles_12[j1] = angles_6[i] / 2
        angles_12[j2] = angles_6[i] / 2

        forces_12[j1] = forces_6[i] / 2
        forces_12[j2] = forces_6[i] / 2

        speeds_12[j1] = speeds_6[i] / 2
        speeds_12[j2] = speeds_6[i] / 2

    return angles_12, forces_12, speeds_12

def compress(angles_12, forces_12, l_r="r"):
    """
    Compress 12 joints to 6 joints.
    """

    if l_r == "r":
        joint_mapping = JOINT_MAPPING_R
    else:
        joint_mapping = JOINT_MAPPING_L

    angles_6 = [0] * 6
    forces_6 = [0] * 6

    for i, (j1, j2) in enumerate(joint_mapping):
        angles_6[i] = round((angles_12[j1] + angles_12[j2]) / 2)
        forces_6[i] = round((forces_12[j1] + forces_12[j2]) / 2)

    return angles_6, forces_6