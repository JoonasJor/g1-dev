import time
import numpy as np
import select
import os
import sys

from inspire import inspire_dds
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient

import inspire.inspire_dds as inspire_dds
import inspire.inspire_defaults as inspire_defaults

from unitree_sdk2py.utils.thread import RecurrentThread

from inspire_sdkpy import inspire_sdk

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import config as Cfg
import g1_joints as Joints

class Controller:
    def __init__(self):
        self.dt = 0.002

class BodyController(Controller):
    def __init__(self):
        super().__init__()

        self.counter = 0

        self.num_motor_body = Cfg.NUM_MOTOR_BODY
        self.kp = Joints.Body.default_kp_list()
        self.kd = Joints.Body.default_kd_list()
        self.mode_machine_ = 0
        self.update_mode_machine_ = False

        self.low_state = None 
        
        self.state_sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.state_sub.Init(self.low_state_handler, 10)

        self.cmd = unitree_hg_msg_dds__LowCmd_()
        self.crc = CRC() 

        self.cmd_pub = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.cmd_pub.Init()

    def low_state_handler(self, msg: LowState_):
        self.low_state = msg
        if self.update_mode_machine_ == False:
            self.mode_machine_ = self.low_state.mode_machine
            self.update_mode_machine_ = True

        self.counter += 1
        if self.counter % 500 == 0:
            #print(self.low_state.imu_state.rpy)
            self.counter = 0

        #for motor in self.low_state.motor_state:
        #    print(motor.q)

    def hold_angles(self):
        current_angles = [motor.q for motor in self.low_state.motor_state]

        while True:
            for i in range(self.num_motor_body):
                self.cmd.mode_pr = 0
                self.cmd.mode_machine = self.mode_machine_
                self.cmd.motor_cmd[i].mode = 1
                self.cmd.motor_cmd[i].tau = 0.0
                self.cmd.motor_cmd[i].dq = 0.0
                self.cmd.motor_cmd[i].kp = self.kp[i]
                self.cmd.motor_cmd[i].kd = self.kd[i]
                self.cmd.motor_cmd[i].q = current_angles[i]

            self.cmd.crc = self.crc.Crc(self.cmd)
            self.cmd_pub.Write(self.cmd)

            time.sleep(self.dt)

            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                input()
                break

    def low_cmd_control(self, target_angles, kp, kd, duration = 10.0, debug = False, wait_for_user_input = True):
        start_angles = [motor.q for motor in self.low_state.motor_state]
        start_time = time.time()

        while True:
            elapsed = time.time() - start_time
            if elapsed >= duration + 1:
                break

            ratio = elapsed / duration
            ratio = np.clip(ratio, 0.0, 1.0)

            for i in range(self.num_motor_body):
                self.cmd.mode_pr = 0
                self.cmd.mode_machine = self.mode_machine_
                self.cmd.motor_cmd[i].mode = 1
                self.cmd.motor_cmd[i].tau = 0.0
                self.cmd.motor_cmd[i].dq = 0.0
                self.cmd.motor_cmd[i].kp = kp[i]
                self.cmd.motor_cmd[i].kd = kd[i]
                q_interpolated = (1.0 - ratio) * start_angles[i] + ratio * target_angles[i]
                self.cmd.motor_cmd[i].q = q_interpolated

            self.cmd.crc = self.crc.Crc(self.cmd)
            self.cmd_pub.Write(self.cmd)

            if debug:
                for i in range(self.num_motor_body):
                    print(
                        f"{i}: "
                        f"current={self.low_state.motor_state[i].q:.5f} "
                        f"target={target_angles[i]:.5f} "
                        f"q={self.cmd.motor_cmd[i].q:.5f} "
                        f"tau={self.cmd.motor_cmd[i].tau} "
                        f"dq={self.cmd.motor_cmd[i].dq} "
                        f"kp={self.cmd.motor_cmd[i].kp} "
                        f"kd={self.cmd.motor_cmd[i].kd}"
                    )   
                print()

            time.sleep(self.dt) 
        
        print("Done.") 

        if wait_for_user_input:
            print("Keeping joints at current position. Press Enter to continue...") 
            self.hold_angles()

    def init_msc(self):
        self.msc = MotionSwitcherClient()
        self.msc.SetTimeout(3.0)
        self.msc.Init()

        status, result = self.msc.CheckMode()

        if result != None:
            while result['name']:
                self.msc.ReleaseMode()
                status, result = self.msc.CheckMode()
                time.sleep(1)

        print(f"{status=}\n{result=}")

class HandController(Controller):
    def __init__(self, l_r = "r"):
        super().__init__()

        self.l_r = l_r

        self.num_motor_fingers = Cfg.NUM_MOTOR_FINGERS

        self.low_state = inspire_defaults.state()

        self.state_sub = ChannelSubscriber(f"rt/inspire_hand/state/{l_r}", inspire_dds.inspire_hand_state)
        self.state_sub.Init(self.low_state_handler, 10)

        self.cmd = inspire_defaults.ctrl()
        self.cmd.mode = 0b0001

        self.cmd_pub = ChannelPublisher(f"rt/inspire_hand/ctrl/{l_r}", inspire_dds.inspire_hand_ctrl)
        self.cmd_pub.Init()

    def low_state_handler(self, msg: inspire_dds.inspire_hand_state):
        self.low_state = msg

    def hold_angles(self):
        current_angles = self.low_state.angle_act

        while True:
            self.cmd.angle_set = current_angles

            self.cmd_pub.Write(self.cmd)

            time.sleep(self.dt)

            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                input()
                break

    def low_cmd_control(self, target_angles, duration = 5.0, wait_for_user_input = True, debug = False):
        start_angles = self.low_state.angle_act
        start_time = time.time()

        while True:
            elapsed = time.time() - start_time
            if elapsed >= duration + 1:
                break

            ratio = elapsed / duration
            ratio = np.clip(ratio, 0.0, 1.0)

            interpolated_angles = [
                round((1.0 - ratio) * s + ratio * t)
                for s, t in zip(start_angles, target_angles)
            ]
            self.cmd.angle_set = interpolated_angles

            self.cmd_pub.Write(self.cmd)

            time.sleep(self.dt) 

            if debug:
                print(f"current={self.low_state.angle_act} \ttarget={target_angles} \tcommand={self.cmd.angle_set}")

        print("Done.")
        print(f"current = {self.low_state.angle_act} target was = {target_angles}")
        print()
        

        if wait_for_user_input:
            print("Keeping joints at current position. Press Enter to continue...") 
            self.hold_angles()