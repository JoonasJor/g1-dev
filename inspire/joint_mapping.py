import g1_joints as Joints

NAMES = ["Pinky", "Ring", "Middle", "Index", "Thumb-bend", "Thumb-rotation"]

def get_joint_mapping(l_r = "r"):
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
        [joints.Little1, joints.Little2], # Pinky
        [joints.Ring1,   joints.Ring2],   # Ring
        [joints.Middle1, joints.Middle2], # Middle
        [joints.Index1,  joints.Index2],  # Index
        [joints.Thumb3,  joints.Thumb4],  # Thumb-bend
        [joints.Thumb1,  joints.Thumb2]   # Thumb-rotation
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
        angles_12[j1.idx] = angles_6[i]
        angles_12[j2.idx] = angles_6[i]

        forces_12[j1.idx] = forces_6[i]
        forces_12[j2.idx] = forces_6[i]

        speeds_12[j1.idx] = speeds_6[i]
        speeds_12[j2.idx] = speeds_6[i]

    return angles_12, forces_12, speeds_12

def compress(angles_12, forces_12, l_r):
    """
    Compress 12 joints to 6 joints.
    """

    joint_mapping = get_joint_mapping(l_r)

    angles_6 = [0] * 6
    forces_6 = [0] * 6

    for i, (j1, j2) in enumerate(joint_mapping):
        angles_6[i] = round((angles_12[j1.idx] + angles_12[j2.idx]) / 2)
        forces_6[i] = round((forces_12[j1.idx] + forces_12[j2.idx]) / 2)

    return angles_6, forces_6