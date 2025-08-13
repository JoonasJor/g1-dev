import time
import sys
import numpy as np
import select

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient

from inspire_sdkpy import inspire_hand_defaut,inspire_dds

import config

# -- G1 body joint parameters --
default_angles_body = np.array([
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

default_kp_body = np.array([
    # left leg
    60, 60, 60, 100, 40, 40, 

    # right leg
    60, 60, 60, 100, 40, 40, 

    # Waist
    60, 40, 40,

    # Left arm
    40, 40, 40, 40, 40, 40, 40,

    # Right arm
    40, 40, 40, 40, 40, 40, 40
])

default_kd_body = np.array([
    # left leg
    1, 1, 1, 2, 1, 1,

    # right leg
    1, 1, 1, 2, 1, 1,

    # Waist
    1, 1, 1,

    # Left arm
    1, 1, 1, 1, 1, 1, 1,

    # Right arm
    1, 1, 1, 1, 1, 1, 1
])

joint_limits_body = [
    (-2.5307, 2.8798),   # 0  L_LEG_HIP_PITCH
    (-0.5236, 2.9671),   # 1  L_LEG_HIP_ROLL
    (-2.7576, 2.7576),   # 2  L_LEG_HIP_YAW
    (-0.087267, 2.8798), # 3  L_LEG_KNEE
    (-0.87267, 0.5236),  # 4  L_LEG_ANKLE_PITCH
    (-0.2618, 0.2618),   # 5  L_LEG_ANKLE_ROLL
    (-2.5307, 2.8798),   # 6  R_LEG_HIP_PITCH
    (-2.9671, 0.5236),   # 7  R_LEG_HIP_ROLL
    (-2.7576, 2.7576),   # 8  R_LEG_HIP_YAW
    (-0.087267, 2.8798), # 9  R_LEG_KNEE
    (-0.87267, 0.5236),  # 10 R_LEG_ANKLE_PITCH
    (-0.2618, 0.2618),   # 11 R_LEG_ANKLE_ROLL
    (-2.618, 2.618),     # 12 WAIST_YAW
    (-0.52, 0.52),       # 13 WAIST_ROLL
    (-0.52, 0.52),       # 14 WAIST_PITCH
    (-3.0892, 2.6704),   # 15 L_SHOULDER_PITCH
    (-1.5882, 2.2515),   # 16 L_SHOULDER_ROLL
    (-2.618, 2.618),     # 17 L_SHOULDER_YAW
    (-1.0472, 2.0944),   # 18 L_ELBOW
    (-1.9722, 1.9722),   # 19 L_WRIST_ROLL
    (-1.6144, 1.6144),   # 20 L_WRIST_PITCH
    (-1.6144, 1.6144),   # 21 L_WRIST_YAW
    (-3.0892, 2.6704),   # 22 R_SHOULDER_PITCH
    (-2.2515, 1.5882),   # 23 R_SHOULDER_ROLL
    (-2.618, 2.618),     # 24 R_SHOULDER_YAW
    (-1.0472, 2.0944),   # 25 R_ELBOW
    (-1.9722, 1.9722),   # 26 R_WRIST_ROLL
    (-1.6144, 1.6144),   # 27 R_WRIST_PITCH
    (-1.6144, 1.6144)    # 28 R_WRIST_YAW
]

# -- Inspire hand joint parameters --
default_angles_hand = [
    # Left fingers
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,

    # Right fingers
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0
]

kp_hand = [
    # Left fingers
    20, 20, 20, 20, 20, 20,
    20, 20, 20, 20, 20, 20,

    # Right fingers
    20, 20, 20, 20, 20, 20,
    20, 20, 20, 20, 20, 20
]

kd_hand = [
    # Left fingers
    1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1,

    # Right fingers
    1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1
]


class Mode:
    PR = 0  # Series Control for Pitch/Roll Joints
    AB = 1  # Parallel Control for A/B Joints

class Custom:
    def __init__(self):
        self.time_ = 0.0
        self.control_dt_ = 0.002  # [2ms]
        self.duration_ = 3.0    # [3 s]
        self.counter_ = 0
        self.mode_pr_ = Mode.PR
        self.mode_machine_ = 0
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()  
        self.low_state = None 
        self.update_mode_machine_ = False
        self.crc = CRC()
        self.num_motor = config.NUM_MOTOR_BODY

    def Init(self):
        self.msc = MotionSwitcherClient()
        self.msc.SetTimeout(3.0)
        self.msc.Init()

        status, result = self.msc.CheckMode()
        print(f"{status=}\n{result=}")

        if result != None:
            while result['name']:
                self.msc.ReleaseMode()
                status, result = self.msc.CheckMode()
                time.sleep(1)

        # create subscriber # 
        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateHandler, 10)

        # create publisher #
        self.lowcmd_publisher_ = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.lowcmd_publisher_.Init()

    def LowStateHandler(self, msg: LowState_):
        #q: types.float32 # Shutdown feedback position information
        #dq: types.float32 # Joint feedback speed
        #ddq: types.float32 # Joint feedback acceleration
        #tau_est: types.float32 # Joint feedback torque

        self.low_state = msg

        #print(self.low_state)
        #for i, motor in enumerate(msg.motor_state):
        #    print(f"{i}: {motor.q=}, {motor.dq=}, {motor.ddq=}, {motor.tau_est=}")

        if self.update_mode_machine_ == False:
            self.mode_machine_ = self.low_state.mode_machine
            self.update_mode_machine_ = True
        
        self.counter_ +=1
        if (self.counter_ % 500 == 0) :
            self.counter_ = 0
            #print(self.low_state.imu_state.rpy)

    def HoldAngles(self):
        current_angles = [motor.q for motor in self.low_state.motor_state]

        while True:
            for i in range(self.num_motor):
                self.low_cmd.mode_pr = Mode.PR
                self.low_cmd.mode_machine = self.mode_machine_
                self.low_cmd.motor_cmd[i].mode = 1
                self.low_cmd.motor_cmd[i].tau = 0.0
                self.low_cmd.motor_cmd[i].dq = 0.0
                self.low_cmd.motor_cmd[i].kp = default_kp_body[i]
                self.low_cmd.motor_cmd[i].kd = default_kd_body[i]
                self.low_cmd.motor_cmd[i].q = current_angles[i]

            self.low_cmd.crc = self.crc.Crc(self.low_cmd)
            self.lowcmd_publisher_.Write(self.low_cmd)

            time.sleep(self.control_dt_)

            # Wait for user input to break the loop
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                input()  # Consume the Enter key
                break

    def LowCmdControl(self, target_angles, kp = default_kp_body, kd = default_kd_body, duration = 10, debug = False, wait_for_user_input = True):
        # tau = Joint target torque  
        # q   = Joint target position
        # dq  = Joint target speed
        # kp  = Joint stiffness coefficient
        # kd  = Joint damping coefficient

        start_angles = [motor.q for motor in self.low_state.motor_state]
        start_time = time.time()

        while True:
            elapsed = time.time() - start_time
            if elapsed >= duration + 1:
                break

            ratio = elapsed / duration
            ratio = np.clip(ratio, 0.0, 1.0)

            for i in range(self.num_motor):
                self.low_cmd.mode_pr = Mode.PR
                self.low_cmd.mode_machine = self.mode_machine_
                self.low_cmd.motor_cmd[i].mode = 1
                self.low_cmd.motor_cmd[i].tau = 0.0
                self.low_cmd.motor_cmd[i].dq = 0.0
                self.low_cmd.motor_cmd[i].kp = kp[i]
                self.low_cmd.motor_cmd[i].kd = kd[i]

                q_interpolated = (1.0 - ratio) * start_angles[i] + ratio * target_angles[i]
                q_clamped = np.clip(q_interpolated, *joint_limits_body[i])
                self.low_cmd.motor_cmd[i].q = q_clamped

            self.low_cmd.crc = self.crc.Crc(self.low_cmd)
            self.lowcmd_publisher_.Write(self.low_cmd)

            if debug:
                for i in range(self.num_motor):
                    print(f"{i}: \tcurrent={self.low_state.motor_state[i].q:.5f} \tcommand={self.low_cmd.motor_cmd[i].q:.5f} \ttarget={target_angles[i]:.5f}")
                print()

            time.sleep(self.control_dt_) 
        
        print("Done.\n") 
        if wait_for_user_input:
            print("Keeping joints at current position. Press Enter to continue...") 
            self.HoldAngles()

    def DemoFloor(self):
        while self.low_state is None:
            print(f"[Demo] Waiting for low_state, {self.low_state=}")
            time.sleep(1) 

        time.sleep(2)
        start_angles = [motor.q for motor in self.low_state.motor_state]

        print("Setting joint angles to zero position...")
        target_angles = [0] * 29
        self.LowCmdControl(target_angles, duration=6)

        print("Moving right leg...")
        target_angles = [motor.q for motor in self.low_state.motor_state]
        target_angles[config.G1JointIndex.RightHipRoll] = np.deg2rad(45)
        target_angles[config.G1JointIndex.RightKnee] = np.deg2rad(90)
        target_angles[config.G1JointIndex.RightAnklePitch] = np.deg2rad(90)
        #kp = default_kp_body.copy()
        #kp[G1JointIndex.WaistPitch] = 80
        self.LowCmdControl(target_angles, duration=6)

        print("Setting joint angles to start position...")
        target_angles = start_angles
        self.LowCmdControl(target_angles, duration=6)

    def DemoStand(self):
        while self.low_state is None:
            print(f"[Demo] Waiting for low_state, {self.low_state=}")
            time.sleep(1) 
        time.sleep(2)

        start_angles = [motor.q for motor in self.low_state.motor_state]

        print("Setting joint angles to default position...")
        default_angles = [0] * 29
        default_angles[config.G1JointIndex.LeftShoulderRoll] = np.deg2rad(30)
        default_angles[config.G1JointIndex.RightShoulderRoll] = np.deg2rad(-30)
        self.LowCmdControl(default_angles, duration=6)

        print("Moving hand...")
        target_angles = [motor.q for motor in self.low_state.motor_state]
        target_angles[config.G1JointIndex.LeftElbow] = np.deg2rad(-90)
        self.LowCmdControl(target_angles, duration=6)       

        print("Setting joint angles to default position...")
        self.LowCmdControl(default_angles, duration=6) 

        print("Setting joint angles to start position...")
        target_angles = start_angles
        self.LowCmdControl(target_angles, duration=6)

if __name__ == '__main__':
    if len(sys.argv)>1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(1, "lo")

    custom = Custom()
    custom.Init()

    if config.START_ON_FLOOR:
        custom.DemoFloor()
    else:
        custom.DemoStand()

    print("Exiting demo.")