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

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import config as Cfg
import g1_joints as Joints
import helpers

class Mode:
    PR = 0  # Series Control for Pitch/Roll Joints
    AB = 1  # Parallel Control for A/B Joints

class Controller:
    # TODO: separate to BodyController and HandController
    def __init__(self):
        self.counter = 0
        self.dt = 0.1

        # Body
        self.num_motor_body = Cfg.NUM_MOTOR_BODY
        self.kp_body = Joints.Body.default_kp_list()
        self.kd_body = Joints.Body.default_kd_list()
        self.mode_machine_ = 0
        self.update_mode_machine_ = False

        self.cmd_body = unitree_hg_msg_dds__LowCmd_()
        self.crc = CRC() 

        self.low_state_body = None 
        
        self.body_state_sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.body_state_sub.Init(self.LowStateHandlerBody, 10)
        self.body_cmd_pub = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.body_cmd_pub.Init()

        # Hand
        # TODO: fix duplicate variables/functions
        self.num_motor_fingers = Cfg.NUM_MOTOR_FINGERS

        self.cmd_hand = inspire_defaults.ctrl()
        self.cmd_hand.mode = 0b0001

        self.low_state_hand_r = inspire_defaults.state()
        self.low_state_hand_l = inspire_defaults.state()

        self.hand_state_sub_r = ChannelSubscriber(f"rt/inspire_hand/state/r", inspire_dds.inspire_hand_state)
        self.hand_state_sub_r.Init(self.LowStateHandlerHand_R, 10)
        self.hand_state_sub_l = ChannelSubscriber(f"rt/inspire_hand/state/l", inspire_dds.inspire_hand_state)
        self.hand_state_sub_l.Init(self.LowStateHandlerHand_L, 10)

        self.hand_cmd_pub_r = ChannelPublisher("rt/inspire_hand/ctrl/r", inspire_dds.inspire_hand_ctrl)
        self.hand_cmd_pub_r.Init()
        self.hand_cmd_pub_l = ChannelPublisher("rt/inspire_hand/ctrl/l", inspire_dds.inspire_hand_ctrl)
        self.hand_cmd_pub_l.Init()

    # region Hand
    def LowStateHandlerHand_R(self, msg: inspire_dds.inspire_hand_state):
        self.low_state_hand_r = msg

    def LowStateHandlerHand_L(self, msg: inspire_dds.inspire_hand_state):
        self.low_state_hand_l = msg

    def HoldAnglesHand(self, l_r = "r"):
        while True:
            # Wait for user input to break the loop
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                input()
                break

    def LowCmdControlHand(self, target_angles, l_r = "r", duration = 5.0, wait_for_user_input = True, debug = False):
        """
        Parameters:
        - target_angles: The desired joint angles for the hand
        - l_r: left or right hand
        - duration: duration for the movement
        - wait_for_user_input: after reaching the target angles, hold the position until user input
        """

        start_angles = self.low_state_hand_r.angle_act
        start_time = time.time()

        print(start_angles)

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
            self.cmd_hand.angle_set = interpolated_angles

            if l_r == "r":
                self.hand_cmd_pub_r.Write(self.cmd_hand)
            else:
                self.hand_cmd_pub_l.Write(self.cmd_hand)

            if debug:
                for i, (s, t, interp) in enumerate(zip(start_angles, target_angles, interpolated_angles)):
                    print(f"{i}: \tcurrent={s:.5f} \ttarget={t:.5f} \tcommand={interp:.5f}")
                print()

            time.sleep(self.dt) 
        
        print("Done.\n") 
        if wait_for_user_input:
            print("Keeping joints at current position. Press Enter to continue...") 
            self.HoldAnglesHand()

    # endregion

    # region Body
    def InitMsc(self):
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

    def LowStateHandlerBody(self, msg: LowState_):
        #q: types.float32 # Shutdown feedback position information
        #dq: types.float32 # Joint feedback speed
        #ddq: types.float32 # Joint feedback acceleration
        #tau_est: types.float32 # Joint feedback torque

        self.low_state_body = msg

        #print(self.low_state)
        #for i, motor in enumerate(msg.motor_state):
        #    print(f"{i}: {motor.q=}, {motor.dq=}, {motor.ddq=}, {motor.tau_est=}")

        if self.update_mode_machine_ == False:
            self.mode_machine_ = self.low_state_body.mode_machine
            self.update_mode_machine_ = True
        
        self.counter +=1
        if (self.counter % 500 == 0) :
            self.counter = 0
            #print(self.low_state.imu_state.rpy)

    def HoldAnglesBody(self):
        current_angles = [motor.q for motor in self.low_state_body.motor_state]

        while True:
            for i in range(self.num_motor_body):
                self.cmd_body.mode_pr = Mode.PR
                self.cmd_body.mode_machine = self.mode_machine_
                self.cmd_body.motor_cmd[i].mode = 1
                self.cmd_body.motor_cmd[i].tau = 0.0
                self.cmd_body.motor_cmd[i].dq = 0.0
                self.cmd_body.motor_cmd[i].kp = self.kp_body[i]
                self.cmd_body.motor_cmd[i].kd = self.kd_body[i]
                self.cmd_body.motor_cmd[i].q = current_angles[i]

            self.cmd_body.crc = self.crc.Crc(self.cmd_body)
            self.body_cmd_pub.Write(self.cmd_body)

            time.sleep(self.dt)

            # Wait for user input to break the loop
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                input()
                break

    def LowCmdControlBody(self, target_angles, kp, kd, duration = 10.0, debug = False, wait_for_user_input = True):
        """
        Parameters:
        - target_angles: The desired joint angles for the hand (rad)
        - kd: Joint damping coefficient
        - kp: Joint stiffness coefficient
        - duration: duration for the movement
        - wait_for_user_input: after reaching the target angles, hold the position until user input
        """
        print(f"{target_angles=}")
        print(f"{kp=}")
        print(f"{kd=}")

        start_angles = [motor.q for motor in self.low_state_body.motor_state]
        start_time = time.time()

        # TODO: add these to joint data
        joint_limits_body = [(-2.5307, 2.8798), (-0.5236, 2.9671), (-2.7576, 2.7576), (-0.087267, 2.8798), (-0.87267, 0.5236), (-0.2618, 0.2618), (-2.5307, 2.8798), (-2.9671, 0.5236), (-2.7576, 2.7576), (-0.087267, 2.8798), (-0.87267, 0.5236), (-0.2618, 0.2618), (-2.618, 2.618), (-0.52, 0.52), (-0.52, 0.52), (-3.0892, 2.6704), (-1.5882, 2.2515), (-2.618, 2.618), (-1.0472, 2.0944), (-1.97222, 1.97222), (-1.61443, 1.61443), (-1.61443, 1.61443), (-3.0892, 2.6704), (-2.2515, 1.5882), (-2.618, 2.618), (-1.0472, 2.0944), (-1.97222, 1.97222), (-1.61443, 1.61443), (-1.61443, 1.61443)]


        while True:
            elapsed = time.time() - start_time
            if elapsed >= duration + 1:
                break

            ratio = elapsed / duration
            ratio = np.clip(ratio, 0.0, 1.0)

            for i in range(self.num_motor_body):
                self.cmd_body.mode_pr = Mode.PR
                self.cmd_body.mode_machine = self.mode_machine_
                self.cmd_body.motor_cmd[i].mode = 1
                self.cmd_body.motor_cmd[i].tau = 0.0 # Joint target torque  
                self.cmd_body.motor_cmd[i].dq = 0.0 # Joint target speed
                self.cmd_body.motor_cmd[i].kp = kp[i] # Joint stiffness coefficient
                self.cmd_body.motor_cmd[i].kd = kd[i] # Joint damping coefficient

                q_interpolated = (1.0 - ratio) * start_angles[i] + ratio * target_angles[i]
                q_clamped = np.clip(q_interpolated, *joint_limits_body[i])
                self.cmd_body.motor_cmd[i].q = q_clamped # Joint target position

            self.cmd_body.crc = self.crc.Crc(self.cmd_body)
            self.body_cmd_pub.Write(self.cmd_body)

            if debug:
                for i in range(self.num_motor_body):
                    # make this multiline
                    print(
                        f"{i}: "
                        f"current={self.low_state_body.motor_state[i].q:.5f} "
                        f"target={target_angles[i]:.5f} "
                        f"q={self.cmd_body.motor_cmd[i].q:.5f} "
                        f"tau={self.cmd_body.motor_cmd[i].tau} "
                        f"dq={self.cmd_body.motor_cmd[i].dq} "
                        f"kp={self.cmd_body.motor_cmd[i].kp} "
                        f"kd={self.cmd_body.motor_cmd[i].kd}"
                    )
                          
                print()

            time.sleep(self.dt) 
        
        print("Done.\n") 
        if wait_for_user_input:
            print("Keeping joints at current position. Press Enter to continue...") 
            self.HoldAnglesBody()
    # endregion

    def DemoFloor(self):
        while self.low_state_body is None:
            print(f"[Demo] Waiting for low_state, {self.low_state_body=}")
            time.sleep(1) 
        time.sleep(2)

        start_angles = [motor.q for motor in self.low_state_body.motor_state]

        print("Setting joints to zero position...")
        angles = [0] * 29
        self.LowCmdControlBody(
            target_angles = angles, 
            kp = self.kp_body, 
            kd = self.kd_body, 
            duration = 6.0
        )

        print("Moving right leg...")
        angles = [motor.q for motor in self.low_state_body.motor_state]
        angles[Joints.Body.RightHipRoll.motor_idx] = np.deg2rad(45)
        angles[Joints.Body.RightKnee.motor_idx] = np.deg2rad(90)
        angles[Joints.Body.RightAnklePitch.motor_idx] = np.deg2rad(90)

        self.LowCmdControlBody(
            target_angles = angles, 
            kp = self.kp_body, 
            kd = self.kd_body, 
            duration = 6.0
        )

        print("Setting joints to start position...")
        self.LowCmdControlBody(
            target_angles = start_angles,
            kp = self.kp_body, 
            kd = self.kd_body, 
            duration = 6.0
        )

    def DemoStand(self):
        while self.low_state_body is None:
            print(f"[Demo] Waiting for low_state, {self.low_state_body=}")
            time.sleep(1) 
        time.sleep(2)

        start_angles = [motor.q for motor in self.low_state_body.motor_state]

        print("Setting joints to default position...")
        self.LowCmdControlBody(
            #target_angles = Joints.Body.default_angles_list(), 
            target_angles = [0] * 29,
            kp = self.kp_body, 
            kd = self.kd_body, 
            duration = 6.0,
            debug=True
        )

        print("Moving arm...")
        angles = [motor.q for motor in self.low_state_body.motor_state]
        angles[Joints.Body.LeftElbow.motor_idx] = np.deg2rad(-90)
        self.LowCmdControlBody(
            target_angles = angles, 
            kp = self.kp_body, 
            kd = self.kd_body, 
            duration = 6.0
        )      

        print("Closing hand...")
        angles = [1000] * 6
        self.LowCmdControlHand(
            target_angles = angles, 
            l_r = "l",
            duration = 3.0
        )

        print("Opening hand...")
        angles = [0] * 6
        self.LowCmdControlHand(
            target_angles = angles, 
            l_r = "l",
            duration = 3.0
        )

        print("Setting joints to start position...")
        self.LowCmdControlBody(
            target_angles = start_angles,
            kp = self.kp_body, 
            kd = self.kd_body, 
            duration = 6.0
        )

    def DemoHands(self):
        for _ in range(3): 
            print("Closing hand...")
            angles = [1000] * 6
            self.LowCmdControlHand(
                target_angles = angles, 
                l_r = "l",
                duration = 3.0
            )

            print("Opening hand...")
            angles = [0] * 6
            self.LowCmdControlHand(
                target_angles = angles, 
                l_r = "l",
                duration = 3.0
            )

if __name__ == '__main__':
    if len(sys.argv)>1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(1, "lo")

    controller = Controller()
    controller.InitMsc()

    controller.DemoHands()

    if Cfg.START_ON_FLOOR:
        controller.DemoFloor()
    else:
        controller.DemoStand()

    print("Exiting demo.")