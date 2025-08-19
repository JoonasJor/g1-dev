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
    mujoco_idx_range: tuple    # Mujoco grid sensor index range
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

class SensorMixin:
    @classmethod
    def from_mujoco_idx(cls, mujoco_idx):
        for sensor in cls:
            if sensor.value.mujoco_idx == mujoco_idx:
                return sensor
        raise ValueError(f"No joint with mujoco_idx={mujoco_idx}")

    @classmethod
    def from_idx(cls, idx):
        for sensor in cls:
            if sensor.value.idx == idx:
                return sensor
        raise ValueError(f"No joint with idx={idx}")
    
    @classmethod
    def mujoco_idx_range_list(cls):
        """Return a list of tuples of mujoco indices in definition order."""
        return [sensor.value.mujoco_idx_range for sensor in cls]
    
    @property
    def mujoco_idx(self):
        return self.value.mujoco_idx_range

    @property
    def idx(self):
        return self.value.idx

class TouchSensor_L(SensorMixin, Enum):
    ThumbForceSensor1     = Sensor(mujoco_idx_range=(159, 254), idx=0)   # palm (96)
    ThumbForceSensor2     = Sensor(mujoco_idx_range=(255, 263), idx=1)   # middle (9)
    ThumbForceSensor3     = Sensor(mujoco_idx_range=(264, 359), idx=2)   # top (96)
    ThumbForceSensor4     = Sensor(mujoco_idx_range=(360, 368), idx=3)   # tip (9)

    IndexForceSensor1     = Sensor(mujoco_idx_range=(369, 448), idx=4)   # palm (80)
    IndexForceSensor2     = Sensor(mujoco_idx_range=(449, 544), idx=5)   # top (96)
    IndexForceSensor3     = Sensor(mujoco_idx_range=(545, 553), idx=6)   # tip (9)

    MiddleForceSensor1    = Sensor(mujoco_idx_range=(554, 633), idx=7)   # palm (80)
    MiddleForceSensor2    = Sensor(mujoco_idx_range=(634, 729), idx=8)   # top (96)
    MiddleForceSensor3    = Sensor(mujoco_idx_range=(730, 738), idx=9)   # tip (9)

    RingForceSensor1      = Sensor(mujoco_idx_range=(739, 818), idx=10)  # palm (80)
    RingForceSensor2      = Sensor(mujoco_idx_range=(819, 914), idx=11)  # top (96)
    RingForceSensor3      = Sensor(mujoco_idx_range=(915, 923), idx=12)  # tip (9)

    LittleForceSensor1    = Sensor(mujoco_idx_range=(924, 1003), idx=13) # palm (80)
    LittleForceSensor2    = Sensor(mujoco_idx_range=(1004, 1099), idx=14)# top (96)
    LittleForceSensor3    = Sensor(mujoco_idx_range=(1100, 1108), idx=15)# tip (9)

    PalmForceSensor       = Sensor(mujoco_idx_range=(1109, 1220), idx=16)# palm (112)


class TouchSensor_R(SensorMixin, Enum):
    ThumbForceSensor1     = Sensor(mujoco_idx_range=(1221, 1316), idx=0) # palm (96)
    ThumbForceSensor2     = Sensor(mujoco_idx_range=(1317, 1325), idx=1) # middle (9)
    ThumbForceSensor3     = Sensor(mujoco_idx_range=(1326, 1421), idx=2) # top (96)
    ThumbForceSensor4     = Sensor(mujoco_idx_range=(1422, 1430), idx=3) # tip (9)

    IndexForceSensor1     = Sensor(mujoco_idx_range=(1431, 1510), idx=4) # palm (80)
    IndexForceSensor2     = Sensor(mujoco_idx_range=(1511, 1606), idx=5) # top (96)
    IndexForceSensor3     = Sensor(mujoco_idx_range=(1607, 1615), idx=6) # tip (9)

    MiddleForceSensor1    = Sensor(mujoco_idx_range=(1616, 1695), idx=7) # palm (80)
    MiddleForceSensor2    = Sensor(mujoco_idx_range=(1696, 1791), idx=8) # top (96)
    MiddleForceSensor3    = Sensor(mujoco_idx_range=(1792, 1800), idx=9) # tip (9)

    RingForceSensor1      = Sensor(mujoco_idx_range=(1801, 1880), idx=10)# palm (80)
    RingForceSensor2      = Sensor(mujoco_idx_range=(1881, 1976), idx=11)# top (96)
    RingForceSensor3      = Sensor(mujoco_idx_range=(1977, 1985), idx=12)# tip (9)

    LittleForceSensor1    = Sensor(mujoco_idx_range=(1986, 2065), idx=13)# palm (80)
    LittleForceSensor2    = Sensor(mujoco_idx_range=(2066, 2161), idx=14)# top (96)
    LittleForceSensor3    = Sensor(mujoco_idx_range=(2162, 2170), idx=15)# tip (9)

    PalmForceSensor       = Sensor(mujoco_idx_range=(2171, 2282), idx=16)# palm (112)
