from enum import Enum
from dataclasses import dataclass

@dataclass(frozen=True)
class Joint:
    mujoco_idx: int   # Mujoco actuator/sensor index
    motor_idx: int    # Ordered index
    default_angle: float
    default_kp: float
    default_kd: float

class JointMixin:
    @classmethod
    def mujoco_idx_list(cls):
        """Return a list of mujoco indices in definition order."""
        return [joint.value.mujoco_idx for joint in cls]
    
    @classmethod
    def default_angles_list(cls):
        """Return a list of default angles in definition order."""
        return [joint.value.default_angle for joint in cls]

    @classmethod
    def default_kp_list(cls):
        """Return a list of kp values in definition order."""
        return [joint.value.default_kp for joint in cls]

    @classmethod
    def default_kd_list(cls):
        """Return a list of kd values in definition order."""
        return [joint.value.default_kd for joint in cls]
    
    @property
    def mujoco_idx(self):
        return self.value.mujoco_idx

    @property
    def motor_idx(self):
        return self.value.motor_idx

    @property
    def default_angle(self):
        return self.value.default_angle

    @property
    def kp(self):
        return self.value.default_kp

    @property
    def kd(self):
        return self.value.default_kd

class Body(JointMixin, Enum):
    # Left leg
    LeftHipPitch        = Joint(mujoco_idx=0,  motor_idx=0,  default_angle=-0.1,    default_kp=60,  default_kd=1)
    LeftHipRoll         = Joint(mujoco_idx=1,  motor_idx=1,  default_angle=0.0,     default_kp=60,  default_kd=1)
    LeftHipYaw          = Joint(mujoco_idx=2,  motor_idx=2,  default_angle=0.0,     default_kp=60,  default_kd=1)
    LeftKnee            = Joint(mujoco_idx=3,  motor_idx=3,  default_angle=0.3,     default_kp=100, default_kd=2)
    LeftAnklePitch      = Joint(mujoco_idx=4,  motor_idx=4,  default_angle=-0.2,    default_kp=40,  default_kd=1)
    LeftAnkleRoll       = Joint(mujoco_idx=5,  motor_idx=5,  default_angle=0.0,     default_kp=40,  default_kd=1)

    # Right leg
    RightHipPitch       = Joint(mujoco_idx=6,  motor_idx=6,  default_angle=-0.1,    default_kp=60,  default_kd=1)
    RightHipRoll        = Joint(mujoco_idx=7,  motor_idx=7,  default_angle=0.0,     default_kp=60,  default_kd=1)
    RightHipYaw         = Joint(mujoco_idx=8,  motor_idx=8,  default_angle=0.0,     default_kp=60,  default_kd=1)
    RightKnee           = Joint(mujoco_idx=9,  motor_idx=9,  default_angle=0.3,     default_kp=100, default_kd=2)
    RightAnklePitch     = Joint(mujoco_idx=10, motor_idx=10, default_angle=-0.2,    default_kp=40,  default_kd=1)
    RightAnkleRoll      = Joint(mujoco_idx=11, motor_idx=11, default_angle=0.0,     default_kp=40,  default_kd=1)

    # Waist
    WaistYaw            = Joint(mujoco_idx=12, motor_idx=12, default_angle=0.0,     default_kp=60,  default_kd=1)
    WaistRoll           = Joint(mujoco_idx=13, motor_idx=13, default_angle=0.0,     default_kp=40,  default_kd=1)
    WaistPitch          = Joint(mujoco_idx=14, motor_idx=14, default_angle=0.0,     default_kp=40,  default_kd=1)

    # Left arm
    LeftShoulderPitch   = Joint(mujoco_idx=15, motor_idx=15, default_angle=0.0,     default_kp=40,  default_kd=1)
    LeftShoulderRoll    = Joint(mujoco_idx=16, motor_idx=16, default_angle=0.52,    default_kp=40,  default_kd=1)
    LeftShoulderYaw     = Joint(mujoco_idx=17, motor_idx=17, default_angle=0.0,     default_kp=40,  default_kd=1)
    LeftElbow           = Joint(mujoco_idx=18, motor_idx=18, default_angle=1.5,     default_kp=40,  default_kd=1)
    LeftWristRoll       = Joint(mujoco_idx=19, motor_idx=19, default_angle=0.0,     default_kp=40,  default_kd=1)
    LeftWristPitch      = Joint(mujoco_idx=20, motor_idx=20, default_angle=0.0,     default_kp=40,  default_kd=1)
    LeftWristYaw        = Joint(mujoco_idx=21, motor_idx=21, default_angle=0.0,     default_kp=40,  default_kd=1)

    # Right arm
    RightShoulderPitch  = Joint(mujoco_idx=34, motor_idx=22, default_angle=0.0,     default_kp=40,  default_kd=1)
    RightShoulderRoll   = Joint(mujoco_idx=35, motor_idx=23, default_angle=0.52,   default_kp=40,  default_kd=1)
    RightShoulderYaw    = Joint(mujoco_idx=36, motor_idx=24, default_angle=0.0,     default_kp=40,  default_kd=1)
    RightElbow          = Joint(mujoco_idx=37, motor_idx=25, default_angle=1.5,     default_kp=40,  default_kd=1)
    RightWristRoll      = Joint(mujoco_idx=38, motor_idx=26, default_angle=0.0,     default_kp=40,  default_kd=1)
    RightWristPitch     = Joint(mujoco_idx=39, motor_idx=27, default_angle=0.0,     default_kp=40,  default_kd=1)
    RightWristYaw       = Joint(mujoco_idx=40, motor_idx=28, default_angle=0.0,     default_kp=40,  default_kd=1)

class Fingers_L(JointMixin, Enum):
    LeftThumb1          = Joint(mujoco_idx=22, motor_idx=0,  default_angle=0.0,     default_kp=2, default_kd=0.5)
    LeftThumb2          = Joint(mujoco_idx=23, motor_idx=1,  default_angle=0.0,     default_kp=2, default_kd=0.5)
    LeftThumb3          = Joint(mujoco_idx=24, motor_idx=2,  default_angle=0.0,     default_kp=2, default_kd=0.5)
    LeftThumb4          = Joint(mujoco_idx=25, motor_idx=3,  default_angle=0.0,     default_kp=2, default_kd=0.5)
    LeftIndex1          = Joint(mujoco_idx=26, motor_idx=4,  default_angle=0.0,     default_kp=2, default_kd=0.5)
    LeftIndex2          = Joint(mujoco_idx=27, motor_idx=5,  default_angle=0.0,     default_kp=2, default_kd=0.5)
    LeftMiddle1         = Joint(mujoco_idx=28, motor_idx=6,  default_angle=0.0,     default_kp=2, default_kd=0.5)
    LeftMiddle2         = Joint(mujoco_idx=29, motor_idx=7,  default_angle=0.0,     default_kp=2, default_kd=0.5)
    LeftRing1           = Joint(mujoco_idx=30, motor_idx=8,  default_angle=0.0,     default_kp=2, default_kd=0.5)
    LeftRing2           = Joint(mujoco_idx=31, motor_idx=9,  default_angle=0.0,     default_kp=2, default_kd=0.5)
    LeftLittle1         = Joint(mujoco_idx=32, motor_idx=10, default_angle=0.0,     default_kp=2, default_kd=0.5)
    LeftLittle2         = Joint(mujoco_idx=33, motor_idx=11, default_angle=0.0,     default_kp=2, default_kd=0.5)

class Fingers_R(JointMixin, Enum):
    RightThumb1         = Joint(mujoco_idx=41, motor_idx=0,  default_angle=0.0,     default_kp=2, default_kd=0.5)
    RightThumb2         = Joint(mujoco_idx=42, motor_idx=1,  default_angle=0.0,     default_kp=2, default_kd=0.5)
    RightThumb3         = Joint(mujoco_idx=43, motor_idx=2,  default_angle=0.0,     default_kp=2, default_kd=0.5)
    RightThumb4         = Joint(mujoco_idx=44, motor_idx=3,  default_angle=0.0,     default_kp=2, default_kd=0.5)
    RightIndex1         = Joint(mujoco_idx=45, motor_idx=4,  default_angle=0.0,     default_kp=2, default_kd=0.5)
    RightIndex2         = Joint(mujoco_idx=46, motor_idx=5,  default_angle=0.0,     default_kp=2, default_kd=0.5)
    RightMiddle1        = Joint(mujoco_idx=47, motor_idx=6,  default_angle=0.0,     default_kp=2, default_kd=0.5)
    RightMiddle2        = Joint(mujoco_idx=48, motor_idx=7,  default_angle=0.0,     default_kp=2, default_kd=0.5)
    RightRing1          = Joint(mujoco_idx=49, motor_idx=8,  default_angle=0.0,     default_kp=2, default_kd=0.5)
    RightRing2          = Joint(mujoco_idx=50, motor_idx=9,  default_angle=0.0,     default_kp=2, default_kd=0.5)
    RightLittle1        = Joint(mujoco_idx=51, motor_idx=10, default_angle=0.0,     default_kp=2, default_kd=0.5)
    RightLittle2        = Joint(mujoco_idx=52, motor_idx=11, default_angle=0.0,     default_kp=2, default_kd=0.5)  

def get_joint_ranges(mj_model, l_r):
    """
    Returns (joint_angle_range, joint_force_range) for the finger joints
    """
    if l_r == "r":
        joint_indices = Fingers_R.mujoco_idx_list()
    else:
        joint_indices = Fingers_L.mujoco_idx_list()

    joint_angle_range = []
    joint_force_range = []

    for idx in joint_indices:
        # offset by 1 to accommodate for (joint_index: 0 , name: floating_base_joint)
        # this offset only applies to worldbody joints. not actuators or sensors.
        joint_angle_range.append(tuple(mj_model.jnt_range[idx + 1]))
        joint_force_range.append(tuple(mj_model.actuator_ctrlrange[idx]))

    return joint_angle_range, joint_force_range
