import time
import numpy as np
import select
import os
import sys

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import config as cfg

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
        self.num_motor = cfg.NUM_MOTOR_BODY

        self.default_kp_body = cfg.JointDefaults.kp_body
        self.default_kd_body = cfg.JointDefaults.kd_body
        self.joint_limits_body = cfg.JointDefaults.angle_limits_body

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
                self.low_cmd.motor_cmd[i].kp = self.default_kp_body[i]
                self.low_cmd.motor_cmd[i].kd = self.default_kd_body[i]
                self.low_cmd.motor_cmd[i].q = current_angles[i]

            self.low_cmd.crc = self.crc.Crc(self.low_cmd)
            self.lowcmd_publisher_.Write(self.low_cmd)

            time.sleep(self.control_dt_)

            # Wait for user input to break the loop
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                input()
                break

    def LowCmdControl(self, target_angles, kp, kd, duration = 10.0, debug = False, wait_for_user_input = True):
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
                q_clamped = np.clip(q_interpolated, *self.joint_limits_body[i])
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

        print("Setting joints to zero position...")
        angles = [0] * 29
        self.LowCmdControl(
            target_angles = angles, 
            kp = self.default_kp_body, 
            kd = self.default_kd_body, 
            duration = 6.0
        )

        print("Moving right leg...")
        angles = [motor.q for motor in self.low_state.motor_state]
        angles[cfg.G1JointIndex.RightHipRoll] = np.deg2rad(45)
        angles[cfg.G1JointIndex.RightKnee] = np.deg2rad(90)
        angles[cfg.G1JointIndex.RightAnklePitch] = np.deg2rad(90)
        #kp = default_kp_body.copy()
        #kp[G1JointIndex.WaistPitch] = 80
        self.LowCmdControl(
            target_angles = angles, 
            kp = self.default_kp_body, 
            kd = self.default_kd_body, 
            duration = 6.0
        )

        print("Setting joints to start position...")
        self.LowCmdControl(
            target_angles = start_angles,
            kp = self.default_kp_body, 
            kd = self.default_kd_body, 
            duration = 6.0
        )

    def DemoStand(self):
        while self.low_state is None:
            print(f"[Demo] Waiting for low_state, {self.low_state=}")
            time.sleep(1) 
        time.sleep(2)

        start_angles = [motor.q for motor in self.low_state.motor_state]

        print("Setting joints to default position...")
        default_angles = [0] * 29
        default_angles[cfg.G1JointIndex.LeftShoulderRoll] = np.deg2rad(30)
        default_angles[cfg.G1JointIndex.RightShoulderRoll] = np.deg2rad(-30)
        self.LowCmdControl(
            target_angles = default_angles, 
            kp = self.default_kp_body, 
            kd = self.default_kd_body, 
            duration = 6.0
        )

        print("Moving hand...")
        angles = [motor.q for motor in self.low_state.motor_state]
        angles[cfg.G1JointIndex.LeftElbow] = np.deg2rad(-90)
        self.LowCmdControl(
            target_angles = angles, 
            kp = self.default_kp_body, 
            kd = self.default_kd_body, 
            duration = 6.0
        )      

        print("Setting joints to default position...")
        self.LowCmdControl(
            target_angles = default_angles, 
            kp = self.default_kp_body, 
            kd = self.default_kd_body, 
            duration = 6.0
        )

        print("Setting joints to start position...")
        self.LowCmdControl(
            target_angles = start_angles,
            kp = self.default_kp_body, 
            kd = self.default_kd_body, 
            duration = 6.0
        )

if __name__ == '__main__':
    if len(sys.argv)>1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(1, "lo")

    custom = Custom()
    custom.Init()

    if cfg.START_ON_FLOOR:
        custom.DemoFloor()
    else:
        custom.DemoStand()

    print("Exiting demo.")