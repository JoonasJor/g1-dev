import g1_joints as Joints

def get_joint_mapping(l_r):
    """
    Map 12 physical joints to 6 Inspire joints for DDS communication
    """

    if l_r == "r":
        joints = Joints.Hand_R
    else:
        joints = Joints.Hand_L

    # TODO: Check correct mapping for Thumb-bend and Thumb-rotation
    # TODO: Add this mapping to Joints class for easier access
    joint_mapping = [
        [joints.Little1.idx, joints.Little2.idx], # Pinky
        [joints.Ring1.idx,   joints.Ring2.idx],   # Ring
        [joints.Middle1.idx, joints.Middle2.idx], # Middle
        [joints.Index1.idx,  joints.Index2.idx],  # Index
        [joints.Thumb3.idx,  joints.Thumb4.idx],  # Thumb-bend
        [joints.Thumb1.idx,  joints.Thumb2.idx]   # Thumb-rotation
    ]

    return joint_mapping

def expand(angles_6, forces_6, speeds_6, l_r):
    """
    Expand 6 joints to 12 joints.
    """

    joint_mapping = get_joint_mapping(l_r)

    angles_12 = [0.0] * 12
    forces_12 = [0.0] * 12
    speeds_12 = [0.0] * 12

    for i, (j1, j2) in enumerate(joint_mapping):
        angles_12[j1] = angles_6[i]
        angles_12[j2] = angles_6[i]

        forces_12[j1] = forces_6[i]
        forces_12[j2] = forces_6[i]

        speeds_12[j1] = speeds_6[i]
        speeds_12[j2] = speeds_6[i]

    return angles_12, forces_12, speeds_12

def compress(angles_12, forces_12, l_r):
    """
    Compress 12 joints to 6 joints.
    """

    joint_mapping = get_joint_mapping(l_r)

    angles_6 = [0] * 6
    forces_6 = [0] * 6

    for i, (j1, j2) in enumerate(joint_mapping):
        angles_6[i] = round((angles_12[j1] + angles_12[j2]) / 2)
        forces_6[i] = round((forces_12[j1] + forces_12[j2]) / 2)

    return angles_6, forces_6