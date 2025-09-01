import time
import numpy as np
import os
import sys
import select

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient

from unitree_sdk2py.idl.default import unitree_go_msg_dds__SportModeState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_

import inspire.inspire_dds as inspire_dds
import inspire.inspire_defaults as inspire_defaults

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import config as Cfg
import g1_joints as Joints

# rt/arm_sdk keeps the joints at last received position
# rt/low_cmd does not do this
# TODO: in bodycontroller, repeat last received command indefinitely, similar to mujoco implementation
# TODO: in unitree_bridge, remove repeating for rt/low_cmd

class Controller:
    def __init__(self):
        self.dt = 0.002

class BodyController(Controller):
    """
    Controls the whole body using DDS topic "rt/lowcmd"

    Do NOT use simultaneously with high-level control
    """

    def __init__(self):
        super().__init__()

        self.counter = 0

        self.num_motor_body = Cfg.NUM_MOTOR_BODY
        self.kp = Joints.Body.default_kp_list()
        self.kd = Joints.Body.default_kd_list()
        self.mode_machine_ = 0
        self.update_mode_machine_ = False
        self.flg_initialized = False

        self.low_state = None 
        
        self.low_state_sub = ChannelSubscriber(Cfg.TOPIC_BODY_LOW_STATE, LowState_)
        self.low_state_sub.Init(self._low_state_handler, 10)

        self.cmd = unitree_hg_msg_dds__LowCmd_()
        self.crc = CRC() 

        self.cmd_pub = ChannelPublisher(Cfg.TOPIC_BODY_CMD, LowCmd_)
        self.cmd_pub.Init()

    def _low_state_handler(self, msg: LowState_):
        self.low_state = msg

        if self.update_mode_machine_ == False:
            self.mode_machine_ = self.low_state.mode_machine
            self.update_mode_machine_ = True

    def low_cmd_control(self, target_angles, kp=None, kd=None, duration=5.0, debug=False):
        """
        Controls the whole body by interpolating to target angles over a duration.

        Args:
            target_angles (list of float): The target joint angles.
            kp (list of float, optional): Joint stiffness coefficients.
            kd (list of float, optional): Joint damping coefficients.
            duration (float, optional): The time over which to interpolate.
            debug (bool, optional): If True, print debug information.
        """

        if not self.flg_initialized:
            print("Not in debug mode.")
            print("Call init_msc() to enable control.")
            return

        if kp is None:
            kp = self.kp
        if kd is None:
            kd = self.kd

        start_angles = [motor.q for motor in self.low_state.motor_state]
        start_time = time.time()

        while True:
            elapsed = time.time() - start_time
            if elapsed >= duration:
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

    def lock_joints(self):
        """
        Lock all joints by indefinitely sending current joint angles.  
        Increased kp is needed to overpower the robot's own weight.
        """

        if not self.flg_initialized:
            print("Not in debug mode.")
            print("Call init_msc() to enable control.")
            return

        print("Locking all joints...")
        print("Press Enter to unlock joints...")

        current_angles = [motor.q for motor in self.low_state.motor_state]

        while True:
            for i in range(self.num_motor_body):
                self.cmd.mode_pr = 0
                self.cmd.mode_machine = self.mode_machine_
                self.cmd.motor_cmd[i].mode = 1
                self.cmd.motor_cmd[i].tau = 0.0
                self.cmd.motor_cmd[i].dq = 0.0
                self.cmd.motor_cmd[i].kp = self.kp[i] * 2
                self.cmd.motor_cmd[i].kd = self.kd[i] * 2
                self.cmd.motor_cmd[i].q = current_angles[i]

            self.cmd.crc = self.crc.Crc(self.cmd)
            self.cmd_pub.Write(self.cmd)

            time.sleep(self.dt)

            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                input()
                break

    def turn_off_motors(self):
        if not self.flg_initialized:
            print("Not in debug mode.")
            print("Call init_msc() to enable control.")
            return
        
        current_angles = [motor.q for motor in self.low_state.motor_state]

        for i in range(self.num_motor_body):
            self.cmd.motor_cmd[i].mode = 0
            self.cmd.motor_cmd[i].q = current_angles[i]

        self.cmd.crc = self.crc.Crc(self.cmd)
        self.cmd_pub.Write(self.cmd)

    def init_msc(self):
        """
        Initializes motion switcher client and enables debug mode.  
        Calling this will disable high-level control.
        """

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

        self.flg_initialized = True

    def disable_debug_mode(self):
        """
        Disable debug mode and enter zero-torque mode.  
        After calling this, do "damping -> locked standing -> regular mode" to enable high-level control.
        """

        # TODO: figure out how other modes work
        self.msc.SelectMode("ai")
        self.flg_initialized = False

class ArmController(Controller):
    """
    Controls only upper body using DDS topic "rt/arm_sdk"

    Can be used simultaneously with high level control (balancing, walking, ...)
    """

    arm_joints = [
        Joints.Body.WaistYaw,
        Joints.Body.WaistRoll,
        Joints.Body.WaistPitch,

        Joints.Body.LeftShoulderPitch,  
        Joints.Body.LeftShoulderRoll,
        Joints.Body.LeftShoulderYaw,    
        Joints.Body.LeftElbow,
        Joints.Body.LeftWristRoll,      
        Joints.Body.LeftWristPitch, 
        Joints.Body.LeftWristYaw,

        Joints.Body.RightShoulderPitch, 
        Joints.Body.RightShoulderRoll,
        Joints.Body.RightShoulderYaw,   
        Joints.Body.RightElbow,
        Joints.Body.RightWristRoll,     
        Joints.Body.RightWristPitch,
        Joints.Body.RightWristYaw
    ]

    arm_sdk_idx = 29

    def __init__(self):
        super().__init__()

        self.kp = Joints.Body.default_kp_list()
        self.kd = Joints.Body.default_kd_list()

        self.low_state = None 
        self.low_state_sub = ChannelSubscriber(Cfg.TOPIC_BODY_LOW_STATE, LowState_)
        self.low_state_sub.Init(self._low_state_handler, 10)

        self.cmd = unitree_hg_msg_dds__LowCmd_()
        self.crc = CRC() 

        self.cmd_pub = ChannelPublisher(Cfg.TOPIC_ARM_SDK, LowCmd_)
        self.cmd_pub.Init()

        self.high_state = unitree_go_msg_dds__SportModeState_()
        self.high_state_sub = ChannelSubscriber(Cfg.TOPIC_BODY_HIGH_STATE, SportModeState_)
        self.high_state_sub.Init(self._high_state_handler, 10)

        self.ctrl_initialized = False

    def _low_state_handler(self, msg: LowState_):

        self.low_state = msg

    def _high_state_handler(self, msg: SportModeState_):
        #TODO: G1 doesnt seem to send high state information. Figure out later.
        self.high_state = msg

    def low_cmd_control(self, target_angles, kp=None, kd=None, duration=5.0, debug=False):
        """
        Controls the upper body by interpolating to target angles over a duration.

        Args:
            target_angles (list of float): Target joint angles, in radians
            kp (list of float, optional): Joint stiffness coefficient
            kd (list of float, optional): Joint damping coefficient
            duration (float, optional): The time over which to interpolate.
            debug (bool, optional): If True, print debug information.
        """

        if kp is None:
            kp = self.kp
        if kd is None:
            kd = self.kd

        start_angles = [motor.q for motor in self.low_state.motor_state]
        start_time = time.time()

        while True:
            elapsed = time.time() - start_time
            if elapsed >= duration:
                break

            ratio = elapsed / duration
            ratio = np.clip(ratio, 0.0, 1.0)

            for joint in self.arm_joints:
                self.cmd.motor_cmd[joint.idx].tau = 0.0
                self.cmd.motor_cmd[joint.idx].dq = 0.0
                self.cmd.motor_cmd[joint.idx].kp = kp[joint.idx]
                self.cmd.motor_cmd[joint.idx].kd = kd[joint.idx]

                # Interpolate the joint angle
                q_interpolated = (1.0 - ratio) * start_angles[joint.idx] + ratio * target_angles[joint.idx]
                self.cmd.motor_cmd[joint.idx].q = q_interpolated

            # Enable upper body control
            self.cmd.motor_cmd[self.arm_sdk_idx].q =  1.0

            self.cmd.crc = self.crc.Crc(self.cmd)
            self.cmd_pub.Write(self.cmd)

            if debug:
                for joint in self.arm_joints:
                    print(
                        f"{joint.idx}: "
                        f"current={self.low_state.motor_state[joint.idx].q:.5f} "
                        f"target={target_angles[joint.idx]:.5f} "
                        f"q={self.cmd.motor_cmd[joint.idx].q:.5f} "
                        f"tau={self.cmd.motor_cmd[joint.idx].tau} "
                        f"dq={self.cmd.motor_cmd[joint.idx].dq} "
                        f"kp={self.cmd.motor_cmd[joint.idx].kp} "
                        f"kd={self.cmd.motor_cmd[joint.idx].kd}"
                    )
                print()

            time.sleep(self.dt) 

        print("Done.") 
        self.ctrl_initialized = True

    def release_control(self, duration=3.0):
        """
        Gradually release control of the arms.
        
        TODO: Implement in mujoco side
        """

        if not self.ctrl_initialized:
            print("Control not initialized. Nothing to release.")
            print("(The arms will violently jerk to default position if low_cmd_control has not been used yet)")
            return

        start_time = time.time()

        while True:
            elapsed = time.time() - start_time
            if elapsed >= duration:
                break

            ratio = elapsed / duration
            ratio = np.clip(ratio, 0.0, 1.0)  

            self.cmd.motor_cmd[self.arm_sdk_idx].q =  (1 - ratio)  

            self.cmd.crc = self.crc.Crc(self.cmd)
            self.cmd_pub.Write(self.cmd) 

class HandController(Controller):
    """
    Controls the Inspire hands using DDS topic "rt/inspire_hand/ctrl"

    Can be used simultaneously with any other controller
    """

    def __init__(self, l_r = "r"):
        super().__init__()

        self.l_r = l_r

        self.num_motor_fingers = Cfg.NUM_MOTOR_FINGERS

        self.low_state = inspire_defaults.state()

        self.state_sub = ChannelSubscriber(f"{Cfg.TOPIC_HAND_STATE}/{l_r}", inspire_dds.inspire_hand_state)
        self.state_sub.Init(self._low_state_handler, 10)

        self.cmd = inspire_defaults.ctrl()
        self.cmd.mode = 0b0001

        self.cmd_pub = ChannelPublisher(f"{Cfg.TOPIC_HAND_CMD}/{l_r}", inspire_dds.inspire_hand_ctrl)
        self.cmd_pub.Init()

    def _low_state_handler(self, msg: inspire_dds.inspire_hand_state):
        self.low_state = msg

    def low_cmd_control(self, target_angles, duration=5.0, debug=False):
        """
        Controls the hand by interpolating to target angles over a duration.

        Args:
            target_angles (list of int): 
                Target joint angles, in normalized units  
                Order: [Pinky, Ring, Middle, Index, Thumb-bend, Thumb-rotation]  
                Range: 0 (closed) to 1000 (open)   
            duration (float, optional): The time over which to interpolate.
            debug (bool, optional): If True, print debug information.
        """

        start_angles = self.low_state.angle_act
        start_time = time.time()

        while True:
            elapsed = time.time() - start_time
            if elapsed >= duration:
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
