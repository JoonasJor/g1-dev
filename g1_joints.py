from enum import Enum
from dataclasses import dataclass

@dataclass(frozen=True)
class Joint:
    mujoco_idx: int   # Mujoco actuator/sensor index
    idx: int    # Ordered index
    default_angle: float
    default_kp: float
    default_kd: float

@dataclass(frozen=True)
class Sensor:
    mujoco_idx: int   # Mujoco actuator/sensor index
    idx: int    # Ordered index

class JointMixin:
    @classmethod
    def get_joint_ranges(cls, mj_model):
        joint_indices = cls.mujoco_idx_list()

        joint_angle_range = []
        joint_force_range = []

        for idx in joint_indices:
            # offset by 1 to accommodate for (joint_index: 0 , name: floating_base_joint)
            # this offset only applies to worldbody joints. not actuators or sensors.
            joint_angle_range.append(tuple(mj_model.jnt_range[idx + 1]))
            joint_force_range.append(tuple(mj_model.actuator_ctrlrange[idx]))

        return joint_angle_range, joint_force_range
    
    @classmethod
    def from_mujoco_idx(cls, mujoco_idx):
        for joint in cls:
            if joint.value.mujoco_idx == mujoco_idx:
                return joint
        raise ValueError(f"No joint with mujoco_idx={mujoco_idx}")

    @classmethod
    def from_idx(cls, idx):
        for joint in cls:
            if joint.value.idx == idx:
                return joint
        raise ValueError(f"No joint with idx={idx}")
    
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
    def idx(self):
        return self.value.idx

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
    LeftHipPitch        = Joint(mujoco_idx=0,  idx=0,  default_angle=-0.1,    default_kp=60,  default_kd=1)
    LeftHipRoll         = Joint(mujoco_idx=1,  idx=1,  default_angle=0.0,     default_kp=60,  default_kd=1)
    LeftHipYaw          = Joint(mujoco_idx=2,  idx=2,  default_angle=0.0,     default_kp=60,  default_kd=1)
    LeftKnee            = Joint(mujoco_idx=3,  idx=3,  default_angle=0.3,     default_kp=100, default_kd=2)
    LeftAnklePitch      = Joint(mujoco_idx=4,  idx=4,  default_angle=-0.2,    default_kp=40,  default_kd=1)
    LeftAnkleRoll       = Joint(mujoco_idx=5,  idx=5,  default_angle=0.0,     default_kp=40,  default_kd=1)

    # Right leg
    RightHipPitch       = Joint(mujoco_idx=6,  idx=6,  default_angle=-0.1,    default_kp=60,  default_kd=1)
    RightHipRoll        = Joint(mujoco_idx=7,  idx=7,  default_angle=0.0,     default_kp=60,  default_kd=1)
    RightHipYaw         = Joint(mujoco_idx=8,  idx=8,  default_angle=0.0,     default_kp=60,  default_kd=1)
    RightKnee           = Joint(mujoco_idx=9,  idx=9,  default_angle=0.3,     default_kp=100, default_kd=2)
    RightAnklePitch     = Joint(mujoco_idx=10, idx=10, default_angle=-0.2,    default_kp=40,  default_kd=1)
    RightAnkleRoll      = Joint(mujoco_idx=11, idx=11, default_angle=0.0,     default_kp=40,  default_kd=1)

    # Waist
    WaistYaw            = Joint(mujoco_idx=12, idx=12, default_angle=0.0,     default_kp=60,  default_kd=1)
    WaistRoll           = Joint(mujoco_idx=13, idx=13, default_angle=0.0,     default_kp=40,  default_kd=1)
    WaistPitch          = Joint(mujoco_idx=14, idx=14, default_angle=0.0,     default_kp=40,  default_kd=1)

    # Left arm
    LeftShoulderPitch   = Joint(mujoco_idx=15, idx=15, default_angle=0.0,     default_kp=40,  default_kd=1)
    LeftShoulderRoll    = Joint(mujoco_idx=16, idx=16, default_angle=0.52,    default_kp=40,  default_kd=1)
    LeftShoulderYaw     = Joint(mujoco_idx=17, idx=17, default_angle=0.0,     default_kp=40,  default_kd=1)
    LeftElbow           = Joint(mujoco_idx=18, idx=18, default_angle=1.5,     default_kp=40,  default_kd=1)
    LeftWristRoll       = Joint(mujoco_idx=19, idx=19, default_angle=0.0,     default_kp=40,  default_kd=1)
    LeftWristPitch      = Joint(mujoco_idx=20, idx=20, default_angle=0.0,     default_kp=40,  default_kd=1)
    LeftWristYaw        = Joint(mujoco_idx=21, idx=21, default_angle=0.0,     default_kp=40,  default_kd=1)

    # Right arm
    RightShoulderPitch  = Joint(mujoco_idx=34, idx=22, default_angle=0.0,     default_kp=40,  default_kd=1)
    RightShoulderRoll   = Joint(mujoco_idx=35, idx=23, default_angle=-0.52,   default_kp=40,  default_kd=1)
    RightShoulderYaw    = Joint(mujoco_idx=36, idx=24, default_angle=0.0,     default_kp=40,  default_kd=1)
    RightElbow          = Joint(mujoco_idx=37, idx=25, default_angle=1.5,     default_kp=40,  default_kd=1)
    RightWristRoll      = Joint(mujoco_idx=38, idx=26, default_angle=0.0,     default_kp=40,  default_kd=1)
    RightWristPitch     = Joint(mujoco_idx=39, idx=27, default_angle=0.0,     default_kp=40,  default_kd=1)
    RightWristYaw       = Joint(mujoco_idx=40, idx=28, default_angle=0.0,     default_kp=40,  default_kd=1)

class Hand_L(JointMixin, Enum):
    LeftThumb1          = Joint(mujoco_idx=22, idx=0,  default_angle=500,     default_kp=10, default_kd=0.5)
    LeftThumb2          = Joint(mujoco_idx=23, idx=1,  default_angle=500,     default_kp=10, default_kd=0.5)
    LeftThumb3          = Joint(mujoco_idx=24, idx=2,  default_angle=500,     default_kp=10, default_kd=0.5)
    LeftThumb4          = Joint(mujoco_idx=25, idx=3,  default_angle=500,     default_kp=10, default_kd=0.5)
    LeftIndex1          = Joint(mujoco_idx=26, idx=4,  default_angle=500,     default_kp=10, default_kd=0.5)
    LeftIndex2          = Joint(mujoco_idx=27, idx=5,  default_angle=500,     default_kp=10, default_kd=0.5)
    LeftMiddle1         = Joint(mujoco_idx=28, idx=6,  default_angle=500,     default_kp=10, default_kd=0.5)
    LeftMiddle2         = Joint(mujoco_idx=29, idx=7,  default_angle=500,     default_kp=10, default_kd=0.5)
    LeftRing1           = Joint(mujoco_idx=30, idx=8,  default_angle=500,     default_kp=10, default_kd=0.5)
    LeftRing2           = Joint(mujoco_idx=31, idx=9,  default_angle=500,     default_kp=10, default_kd=0.5)
    LeftLittle1         = Joint(mujoco_idx=32, idx=10, default_angle=500,     default_kp=10, default_kd=0.5)
    LeftLittle2         = Joint(mujoco_idx=33, idx=11, default_angle=500,     default_kp=10, default_kd=0.5)

class Hand_R(JointMixin, Enum):
    RightThumb1         = Joint(mujoco_idx=41, idx=0,  default_angle=500,     default_kp=10, default_kd=0.5)
    RightThumb2         = Joint(mujoco_idx=42, idx=1,  default_angle=500,     default_kp=10, default_kd=0.5)
    RightThumb3         = Joint(mujoco_idx=43, idx=2,  default_angle=500,     default_kp=10, default_kd=0.5)
    RightThumb4         = Joint(mujoco_idx=44, idx=3,  default_angle=500,     default_kp=10, default_kd=0.5)
    RightIndex1         = Joint(mujoco_idx=45, idx=4,  default_angle=500,     default_kp=10, default_kd=0.5)
    RightIndex2         = Joint(mujoco_idx=46, idx=5,  default_angle=500,     default_kp=10, default_kd=0.5)
    RightMiddle1        = Joint(mujoco_idx=47, idx=6,  default_angle=500,     default_kp=10, default_kd=0.5)
    RightMiddle2        = Joint(mujoco_idx=48, idx=7,  default_angle=500,     default_kp=10, default_kd=0.5)
    RightRing1          = Joint(mujoco_idx=49, idx=8,  default_angle=500,     default_kp=10, default_kd=0.5)
    RightRing2          = Joint(mujoco_idx=50, idx=9,  default_angle=500,     default_kp=10, default_kd=0.5)
    RightLittle1        = Joint(mujoco_idx=51, idx=10, default_angle=500,     default_kp=10, default_kd=0.5)
    RightLittle2        = Joint(mujoco_idx=52, idx=11, default_angle=500,     default_kp=10, default_kd=0.5)

class TouchSensor_L(JointMixin, Enum):
    LeftThumbForceSensor1     = Sensor(mujoco_idx=159, idx=0)
    LeftThumbForceSensor2     = Sensor(mujoco_idx=160, idx=1)
    LeftThumbForceSensor3     = Sensor(mujoco_idx=161, idx=2)
    LeftThumbForceSensor4     = Sensor(mujoco_idx=162, idx=3)
    LeftIndexForceSensor1     = Sensor(mujoco_idx=163, idx=4)
    LeftIndexForceSensor2     = Sensor(mujoco_idx=164, idx=5)
    LeftIndexForceSensor3     = Sensor(mujoco_idx=165, idx=6)
    LeftMiddleForceSensor1    = Sensor(mujoco_idx=166, idx=7)
    LeftMiddleForceSensor2    = Sensor(mujoco_idx=167, idx=8)
    LeftMiddleForceSensor3    = Sensor(mujoco_idx=168, idx=9)
    LeftRingForceSensor1      = Sensor(mujoco_idx=169, idx=10)
    LeftRingForceSensor2      = Sensor(mujoco_idx=170, idx=11)
    LeftRingForceSensor3      = Sensor(mujoco_idx=171, idx=12)
    LeftLittleForceSensor1    = Sensor(mujoco_idx=172, idx=13)
    LeftLittleForceSensor2    = Sensor(mujoco_idx=173, idx=14)
    LeftLittleForceSensor3    = Sensor(mujoco_idx=174, idx=15)
    LeftPalmForceSensor       = Sensor(mujoco_idx=175, idx=16)

class TouchSensor_R(JointMixin, Enum):
    RightThumbForceSensor1    = Sensor(mujoco_idx=176, idx=0)
    RightThumbForceSensor2    = Sensor(mujoco_idx=177, idx=1)
    RightThumbForceSensor3    = Sensor(mujoco_idx=178, idx=2)
    RightThumbForceSensor4    = Sensor(mujoco_idx=179, idx=3)
    RightIndexForceSensor1    = Sensor(mujoco_idx=180, idx=4)
    RightIndexForceSensor2    = Sensor(mujoco_idx=181, idx=5)
    RightIndexForceSensor3    = Sensor(mujoco_idx=182, idx=6)
    RightMiddleForceSensor1   = Sensor(mujoco_idx=183, idx=7)
    RightMiddleForceSensor2   = Sensor(mujoco_idx=184, idx=8)
    RightMiddleForceSensor3   = Sensor(mujoco_idx=185, idx=9)
    RightRingForceSensor1     = Sensor(mujoco_idx=186, idx=10)
    RightRingForceSensor2     = Sensor(mujoco_idx=187, idx=11)
    RightRingForceSensor3     = Sensor(mujoco_idx=188, idx=12)
    RightLittleForceSensor1   = Sensor(mujoco_idx=189, idx=13)
    RightLittleForceSensor2   = Sensor(mujoco_idx=190, idx=14)
    RightLittleForceSensor3   = Sensor(mujoco_idx=191, idx=15)
    RightPalmForceSensor      = Sensor(mujoco_idx=192, idx=16)

def get_joint_ranges(mj_model, l_r):
    """
    Returns (joint_angle_range, joint_force_range) for the finger joints
    """
    if l_r == "r":
        joint_indices = Hand_R.mujoco_idx_list()
    else:
        joint_indices = Hand_L.mujoco_idx_list()

    joint_angle_range = []
    joint_force_range = []

    for idx in joint_indices:
        # offset by 1 to accommodate for (joint_index: 0 , name: floating_base_joint)
        # this offset only applies to worldbody joints. not actuators or sensors.
        joint_angle_range.append(tuple(mj_model.jnt_range[idx + 1]))
        joint_force_range.append(tuple(mj_model.actuator_ctrlrange[idx]))

    return joint_angle_range, joint_force_range
