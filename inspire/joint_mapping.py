import g1_joints as Joints

# This maps 12 physical joints to 6 Inspire joints for DDS communication
# TODO: Check correct mapping for Thumb-bend and Thumb-rotation
JOINT_MAPPING_R = [
    [Joints.Hand_R.RightLittle1.idx, Joints.Hand_R.RightLittle2.idx], # Pinky
    [Joints.Hand_R.RightRing1.idx,   Joints.Hand_R.RightRing2.idx],   # Ring
    [Joints.Hand_R.RightMiddle1.idx, Joints.Hand_R.RightMiddle2.idx], # Middle
    [Joints.Hand_R.RightIndex1.idx,  Joints.Hand_R.RightIndex2.idx],  # Index
    [Joints.Hand_R.RightThumb3.idx,  Joints.Hand_R.RightThumb4.idx],  # Thumb-bend
    [Joints.Hand_R.RightThumb1.idx,  Joints.Hand_R.RightThumb2.idx]   # Thumb-rotation
]

JOINT_MAPPING_L = [
    [Joints.Hand_L.LeftLittle1.idx,  Joints.Hand_L.LeftLittle2.idx],  # Pinky
    [Joints.Hand_L.LeftRing1.idx,    Joints.Hand_L.LeftRing2.idx],    # Ring
    [Joints.Hand_L.LeftMiddle1.idx,  Joints.Hand_L.LeftMiddle2.idx],  # Middle
    [Joints.Hand_L.LeftIndex1.idx,   Joints.Hand_L.LeftIndex2.idx],   # Index
    [Joints.Hand_L.LeftThumb3.idx,   Joints.Hand_L.LeftThumb4.idx],   # Thumb-bend
    [Joints.Hand_L.LeftThumb1.idx,   Joints.Hand_L.LeftThumb2.idx]    # Thumb-rotation
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
        angles_12[j1] = angles_6[i] #/ 2
        angles_12[j2] = angles_6[i] #/ 2

        forces_12[j1] = forces_6[i] #/ 2
        forces_12[j2] = forces_6[i] #/ 2

        speeds_12[j1] = speeds_6[i] #/ 2
        speeds_12[j2] = speeds_6[i] #/ 2

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
        #angles_6[i] = round((angles_12[j1] + angles_12[j2]) / 2)
        #forces_6[i] = round((forces_12[j1] + forces_12[j2]) / 2)

        angles_6[i] = round((angles_12[j1] + angles_12[j2]))
        forces_6[i] = round((forces_12[j1] + forces_12[j2]))

    return angles_6, forces_6