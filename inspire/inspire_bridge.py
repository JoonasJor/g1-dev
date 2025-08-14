import traceback
import numpy as np

from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelPublisher
from unitree_sdk2py.utils.thread import RecurrentThread

import inspire_dds
import inspire_defaults

import config as cfg

TOPIC_CMD = "rt/inspire_hand/ctrl"
TOPIC_STATE = "rt/inspire_hand/state"
TOPIC_TOUCH = "rt/inspire_hand/touch"

class InspireBridge():
    def __init__(self, mj_model, mj_data, l_r = "r"): # l = left hand, r = right hand
        self.mj_model = mj_model
        self.mj_data = mj_data
        self.dt = self.mj_model.opt.timestep

        self.num_motor = cfg.NUM_MOTOR_BODY # TODO: check this

        # Inspire message
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
            print("[HandCmdHandler] mj_data is None")
            return
        
        # TODO: modify to work with 6 dof
        # TODO: make class similar to inspire_subscribe.py to handle two hands and to convert between inspire_hand_ctrl and MotorCmd_
        test_idx = [0,0,1,1,2,2,3,3,4,4,5,5] # for testing
        try:
            for i, joint_idx in enumerate(cfg.HandJointIndex_R.idx_list()):
                self.mj_data.ctrl[joint_idx] = (
                    # TODO: figure out how to properly calculate this
                    msg.force_set[test_idx[i]]
                    + 20 # is this needed?
                    * (msg.angle_set[test_idx[i]] / 1000 - self.mj_data.sensordata[joint_idx])
                    + 1 # is this needed?
                    * (
                        msg.speed_set[test_idx[i]]
                        - self.mj_data.sensordata[joint_idx + self.num_motor]
                    )
                )
        except Exception as e:
            print(f"[HandCmdHandler] error: {type(e).__name__}: {e}")
            print(f"{len(self.mj_data.ctrl)=}")
            print(f"{self.num_motor=}")
            traceback.print_exc()

    def PublishHandState(self):
        if self.mj_data is None:
            print("[PublishHandState] mj_data is None")
            return
        
        test_idx = [0,0,1,1,2,2,3,3,4,4,5,5] # for testing
        for i, joint_idx in enumerate(cfg.HandJointIndex_R.idx_list()):
            q_index = i + joint_idx
            dq_index = i + self.num_motor + joint_idx
            tau_index = i + 2 * self.num_motor + joint_idx

            try:
                # for testing
                self.hand_state.angle_act[test_idx[i]] = np.clip(int(self.mj_data.sensordata[q_index] * 1000), 0, 1000)
                self.hand_state.force_act[test_idx[i]] = np.clip(int(self.mj_data.sensordata[tau_index] * 1000), 0, 1000)
            except IndexError as e:
                #return
                print(f"[PublishHandState] error: {e} - {i=}, {q_index=}, {dq_index=}, {tau_index=}")
                print(f"{len(self.hand_state.angle_act)=}")
                print(f"{len(self.mj_data.sensordata)=}")
                print(f"{self.num_motor=}")
                
            except Exception as e:
                print(f"[PublishHandState] error: {type(e).__name__}: {e}")  
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