import traceback
import numpy as np
import os
import sys

from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelPublisher
from unitree_sdk2py.utils.thread import RecurrentThread

import inspire.inspire_dds as inspire_dds
import inspire.inspire_defaults as inspire_defaults
import inspire.joint_mapping as joint_mapping

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import helpers as helpers
import config as Cfg
import g1_joints as Joints

TOPIC_CMD = "rt/inspire_hand/ctrl"
TOPIC_STATE = "rt/inspire_hand/state"
TOPIC_TOUCH = "rt/inspire_hand/touch"

class InspireBridge():
    def __init__(self, mj_model, mj_data, l_r = "r"):
        self.mj_model = mj_model
        self.mj_data = mj_data
        self.dt = self.mj_model.opt.timestep

        self.num_motor = Cfg.NUM_MOTOR_BODY + Cfg.NUM_MOTOR_FINGERS
        
        if l_r == "r":
            self.joint_angle_range, self.joint_force_range = Joints.Hand_R.get_joint_ranges(mj_model)
            self.kp = Joints.Hand_R.default_kp_list()
            self.kd = Joints.Hand_R.default_kd_list()
        else:
            self.joint_angle_range, self.joint_force_range = Joints.Hand_L.get_joint_ranges(mj_model)
            self.kp = Joints.Hand_L.default_kp_list()
            self.kd = Joints.Hand_L.default_kd_list()

        if l_r == "r":
            for i, angle in enumerate(self.joint_angle_range):
                print(f"{Joints.Hand_R.from_idx(i)}: {angle}")

        # Inspire message
        self.l_r = l_r
        self.hand_state = inspire_defaults.state()
        self.hand_state_pub = ChannelPublisher(f"{TOPIC_STATE}/{l_r}", inspire_dds.inspire_hand_state)
        self.hand_state_pub.Init()
        self.hand_state_thread = RecurrentThread(
            interval=self.dt, target=self.PublishHandState, name=f"sim_handstate_{l_r}"
        )
        self.hand_state_thread.Start()

        self.hand_touch = inspire_defaults.touch()
        self.hand_touch_pub = ChannelPublisher(f"{TOPIC_TOUCH}/{l_r}", inspire_dds.inspire_hand_touch)
        self.hand_touch_pub.Init()
        self.hand_touch_thread = RecurrentThread(
            interval=0.2, target=self.PublishHandTouch, name=f"sim_handstate_{l_r}" # remember to change interval back to dt
        )
        self.hand_touch_thread.Start()

        self.hand_ctrl_sub = ChannelSubscriber(f"{TOPIC_CMD}/{l_r}", inspire_dds.inspire_hand_ctrl)
        self.hand_ctrl_sub.Init(self.HandCmdHandler, 10)

    def HandCmdHandler(self, msg: inspire_dds.inspire_hand_ctrl):
        if self.mj_data is None:
            print(f"[HandCmdHandler_{self.l_r}] mj_data is None")
            return
        
        if self.l_r == "r":
            joint_indice = Joints.Hand_R.mujoco_idx_list()
        else:
            joint_indice = Joints.Hand_L.mujoco_idx_list()

        try:
            # Convert inspire dds message to mujoco control command
            # 1. Expand 6 joints to 12 joints
            angles_scaled_12, forces_scaled_12, speeds_scaled_12 = joint_mapping.expand(msg.angle_set, msg.force_set, msg.speed_set, self.l_r)

            angles_12 = [0.0] * 12
            forces_12 = [0.0] * 12
            speeds_12 = [0.0] * 12

            for i, joint_idx in enumerate(joint_indice):
                # 2. Convert angles from (0 - 1000) to radians and forces from (-4000 - 4000) to N
                angles_12[i] = helpers.num_to_range(angles_scaled_12[i], 0, 1000, *self.joint_angle_range[i])
                forces_12[i] = helpers.num_to_range(forces_scaled_12[i], -4000, 4000, *self.joint_force_range[i])
                speeds_12[i] = 0 # TODO: figure out later

                # 3. Set the mujoco control command
                control = (
                    forces_12[i] + self.kp[i]
                    * (angles_12[i] - self.mj_data.sensordata[joint_idx]) + self.kd[i]
                    * (speeds_12[i] - self.mj_data.sensordata[joint_idx + self.num_motor])
                )
                self.mj_data.ctrl[joint_idx] = control

                #print(f"[HandCmdHandler_{self.l_r}] {joint_idx=} {angle_set=} {force_set=} {speed_set=}")
                #print(f"[HandCmdHandler_{self.l_r}] {joint_idx=} {self.mj_data.ctrl[joint_idx]=} {self.mj_data.sensordata[joint_idx]=}")

            if False:
                finger1 = Joints.Hand_R.RightLittle1
                finger2 = Joints.Hand_R.RightLittle2
                print(f"[HandCmdHandler_{self.l_r}] {finger1.mujoco_idx} {finger2.mujoco_idx} \
                        \ncurrent: {self.mj_data.sensordata[finger1.mujoco_idx]:.3f} {self.mj_data.sensordata[finger2.mujoco_idx]:.3f} \
                        \ntarget scaled: {angles_scaled_12[finger1.idx]} {angles_scaled_12[finger2.idx]} \
                        \ntarget rad: {angles_12[finger1.idx]:.3f} {angles_12[finger2.idx]:.3f} \
                        \nangle range rad: {self.joint_angle_range[finger1.idx]} {self.joint_angle_range[finger2.idx]} \
                        \nforce range: {self.joint_force_range[finger1.idx]} {self.joint_force_range[finger2.idx]} \
                        \nctrl: {control:.3f}")

        except Exception as e:
            print(f"[HandCmdHandler_{self.l_r}] error: {type(e).__name__}: {e}")
            print(f"{len(self.mj_data.ctrl)=}")
            print(f"{self.num_motor=}")
            traceback.print_exc()

    def PublishHandState(self):
        if self.mj_data is None:
            print(f"[PublishHandState_{self.l_r}] mj_data is None")
            return
        
        if self.l_r == "r":
            joints = Joints.Hand_R
        else:
            joints = Joints.Hand_L
        
        angles_12 = [0.0] * 12
        forces_12 = [0.0] * 12
        speeds_12 = [0.0] * 12

        for joint in joints:
            q_index = joint.mujoco_idx
            dq_index = joint.mujoco_idx + self.num_motor
            tau_index = joint.mujoco_idx + 2 * self.num_motor

            try:
                angles_12[joint.idx] = self.mj_data.sensordata[q_index]
                forces_12[joint.idx] = self.mj_data.sensordata[dq_index]
                speeds_12[joint.idx] = self.mj_data.sensordata[tau_index]

            except IndexError as e:
                print(f"[PublishHandState_{self.l_r}] error: {e} - {i=}, {q_index=}, {dq_index=}, {tau_index=}")
                print(f"{len(angles_12)=}")
                print(f"{len(self.mj_data.sensordata)=}")
                print(f"{self.num_motor=}")
            except Exception as e:
                print(f"[PublishHandState_{self.l_r}] error: {type(e).__name__}: {e}")

        # Convert mujoco sensor data to inspire DDS format
        # 1. Convert angles from radians to (0 - 1000) and forces from N to (-4000 - 4000)
        angles_scaled_12 = []
        for i, angle in enumerate(angles_12):
            scaled = helpers.num_to_range(angle, *self.joint_angle_range[i], 0, 1000)
            angles_scaled_12.append(scaled)

        forces_scaled_12 = []
        for i, force in enumerate(forces_12):
            scaled = helpers.num_to_range(force, *self.joint_force_range[i], -4000, 4000)
            forces_scaled_12.append(scaled)

        # 2. Compress 12 joints to 6 joints
        angles_scaled_6, forces_scaled_6 = joint_mapping.compress(angles_scaled_12, forces_scaled_12, self.l_r)

        # 3. Fill the inspire hand state message
        self.hand_state.angle_act = angles_scaled_6
        self.hand_state.force_act = forces_scaled_6

        #print(f"[PublishHandState_{self.l_r}] {forces_scaled_6=}")
        #print(f"[PublishHandState_{self.l_r}] {forces_scaled_12=}")
        #print(f"[PublishHandState_{self.l_r}] {angles_scaled_6=}")
        #print(f"[PublishHandState_{self.l_r}] {angles_scaled_12=}")

        # 4. Publish the hand state
        self.hand_state_pub.Write(self.hand_state)
            
    def PublishHandTouch(self):
        if self.mj_data is None:
            print("[PublishHandTouch] mj_data is None")
            return
        
        if self.l_r == "r":
            sensors = Joints.TouchSensor_R
        else:
            sensors = Joints.TouchSensor_L
        
        for sensor in sensors:
            start_idx, end_idx = sensor.mujoco_idx
            print(f"{sensor}: {self.mj_data.sensordata[start_idx:end_idx]}")

        # dds range = 0-4095

        return
