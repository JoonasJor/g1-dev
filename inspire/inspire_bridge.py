import traceback
import numpy as np

from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelPublisher
from unitree_sdk2py.utils.thread import RecurrentThread

import inspire.inspire_dds as inspire_dds
import inspire.inspire_defaults as inspire_defaults
import inspire.joint_mapping as joint_mapping
import inspire.conversions as conversions

import config as cfg

TOPIC_CMD = "rt/inspire_hand/ctrl"
TOPIC_STATE = "rt/inspire_hand/state"
TOPIC_TOUCH = "rt/inspire_hand/touch"

class InspireBridge():
    def __init__(self, mj_model, mj_data, l_r = "r"): # l = left hand, r = right hand
        self.mj_model = mj_model
        self.mj_data = mj_data
        self.dt = self.mj_model.opt.timestep

        self.num_motor = cfg.NUM_MOTOR_BODY + cfg.NUM_MOTOR_HANDS
        
        self.angle_range, self.force_range = joint_mapping.get_joint_limits(mj_model, l_r)
        self.kp = cfg.JointDefaults.kp_hand
        self.kd = cfg.JointDefaults.kd_hand

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
            interval=self.dt, target=self.PublishHandTouch, name=f"sim_handstate_{l_r}"
        )
        self.hand_touch_thread.Start()

        self.hand_ctrl_sub = ChannelSubscriber(f"{TOPIC_CMD}/{l_r}", inspire_dds.inspire_hand_ctrl)
        self.hand_ctrl_sub.Init(self.HandCmdHandler, 10)

    def HandCmdHandler(self, msg: inspire_dds.inspire_hand_ctrl):
        if self.mj_data is None:
            print(f"[HandCmdHandler_{self.l_r}] mj_data is None")
            return
        
        if self.l_r == "r":
            joint_indice = cfg.HandJointIndex_R.idx_list()
        else:
            joint_indice = cfg.HandJointIndex_L.idx_list()

        try:
            # Convert inspire dds message to mujoco control command
            # 1. Expand 6 joints to 12 joints
            angles_12, forces_12, speeds_12 = joint_mapping.dds_to_mujoco(msg.angle_set, msg.force_set, msg.speed_set, self.l_r)

            for i, joint_idx in enumerate(joint_indice):
                # 2. Convert angles and forces from (0 - 1000) to radians and from (-4000 - 4000) to N
                angle_set = conversions.scaled_to_radian(angles_12[i], *self.angle_range[i])
                force_set = conversions.scaled_to_force(forces_12[i], *self.force_range[i])
                speed_set = 0 # TODO: figure out later

                if self.l_r == "r":
                    print(f"[HandCmdHandler_{self.l_r}] {joint_idx=} {angle_set=} {force_set=} {speed_set=}")

                # 3. Set the mujoco control command
                self.mj_data.ctrl[joint_idx] = (
                    force_set + self.kp[i]
                    * (angle_set - self.mj_data.sensordata[joint_idx]) + self.kd[i]
                    * (speed_set - self.mj_data.sensordata[joint_idx + self.num_motor])
                )

                if self.l_r == "r":
                    print(f"[HandCmdHandler_{self.l_r}] {joint_idx=} {self.mj_data.ctrl[joint_idx]=} {self.mj_data.sensordata[joint_idx]=}")

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
            joint_indice = cfg.HandJointIndex_R.idx_list()
        else:
            joint_indice = cfg.HandJointIndex_L.idx_list()
        
        angles_12 = [0.0] * 12
        forces_12 = [0.0] * 12
        speeds_12 = [0.0] * 12

        for i, joint_idx in enumerate(joint_indice):
            q_index = joint_idx
            dq_index = joint_idx + self.num_motor
            tau_index = joint_idx + 2 * self.num_motor  

            try:
                angles_12[i] = self.mj_data.sensordata[q_index]
                forces_12[i] = self.mj_data.sensordata[dq_index]
                speeds_12[i] = self.mj_data.sensordata[tau_index]

            except IndexError as e:
                print(f"[PublishHandState_{self.l_r}] error: {e} - {i=}, {q_index=}, {dq_index=}, {tau_index=}")
                print(f"{len(angles_12)=}")
                print(f"{len(self.mj_data.sensordata)=}")
                print(f"{self.num_motor=}")

        # Convert mujoco sensor data to inspire DDS format
        # 1. Convert angles and forces from radians to (0 - 1000) and from N to (-4000 - 4000)
        angles_scaled_12 = []
        for i, angle in enumerate(angles_12):
            scaled = conversions.radian_to_scaled(angle, *self.angle_range[i])
            angles_scaled_12.append(scaled)

        force_scaled_12 = []
        for i, force in enumerate(forces_12):
            scaled = conversions.force_to_scaled(force, *self.force_range[i])
            force_scaled_12.append(scaled)

        # 2. Compress 12 joints to 6 joints
        angles_scaled_6, forces_scaled_6 = joint_mapping.mujoco_to_dds(angles_scaled_12, force_scaled_12, self.l_r)

        # 3. Fill the inspire hand state message
        self.hand_state.angle_act = angles_scaled_6
        self.hand_state.force_act = forces_scaled_6

        # 4. Publish the hand state
        self.hand_state_pub.Write(self.hand_state)
            
    def PublishHandTouch(self):
        if self.mj_data is None:
            print("[PublishHandTouch] mj_data is None")
            return
        
        # TODO: msg = self.mj_data.sensordata[force_sensor_index]

        return

        for (var, addr, length, size) in HAND_TOUCH_DATA:
            value = getattr(msg, var)
            if value is not None:
                matrix = np.array(value).reshape(size)
                self.hand_touch[var]=matrix

        self.hand_state_pub.Write(self.hand_state)

        return

        for i, joint_idx in enumerate(cfg.HandJointIndex_R.idx_list()):
            force_sensor_index = i + 3 * self.num_motor + joint_idx

            try:
                # for testing
                # TODO: check whether left_little_force_sensor_1 corresponds to tip top or palm
                #self.hand_touch_r[test_idx[i]] = self.mj_data.sensordata[force_sensor_index]
                pass
            except IndexError as e:
                print(f"[PublishHandTouch] error: {e} - {i=}, {force_sensor_index=}")
                print(f"{len(self.hand_state.angle_act)=}")
                print(f"{len(self.mj_data.sensordata)=}")
                print(f"{self.num_motor=}")
                
            except Exception as e:
                print(f"[PublishHandTouch] error: {type(e).__name__}: {e}")  
        self.hand_state_pub.Write(self.hand_state)