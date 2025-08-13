from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize

from inspire_sdkpy import inspire_hand_defaut,inspire_dds

import numpy as np
import time
import sys
import threading

class DDSHandler():
   
    def __init__(self,sub_touch=True,LR='r'):
        super().__init__()  # 调用父类的 __init__ 方法

        self.data=inspire_hand_defaut.data_sheet
        if sub_touch:
            self.sub_touch = ChannelSubscriber("rt/inspire_hand/touch/"+LR, inspire_dds.inspire_hand_touch)
            self.sub_touch.Init(self.update_data_touch, 10)
        
        self.sub_states = ChannelSubscriber("rt/inspire_hand/state/"+LR, inspire_dds.inspire_hand_state)
        self.sub_states.Init(self.update_data_state, 10)

        self.ctrl_states = ChannelSubscriber("rt/inspire_hand/ctrl/"+LR, inspire_dds.inspire_hand_ctrl)
        self.ctrl_states.Init(self.ctrl_state, 10)

        self.touch={}
        self.states={}
        self.data_touch_lock = threading.Lock()
        self.data_state_lock = threading.Lock()

    def ctrl_state(self, msg:inspire_dds.inspire_hand_ctrl):
        #print(msg)
        pass

    # 更新图形的函数
    def update_data_touch(self,msg:inspire_dds.inspire_hand_touch):
            print(f"{msg.fingerone_top_touch=}")
            
    def update_data_state(self,states_msg:inspire_dds.inspire_hand_state):
        self.states= {
            'POS_ACT': states_msg.pos_act,
            'ANGLE_ACT': states_msg.angle_act,
            'FORCE_ACT': states_msg.force_act,
            'CURRENT': states_msg.current,
            'ERROR': states_msg.err,
            'STATUS': states_msg.status,
            'TEMP': states_msg.temperature
        }
        print(f"{states_msg.angle_act=}")

    def read(self):
        with self.data_state_lock and self.data_touch_lock:
            print(f"{self.states=}")
        

if __name__ == '__main__':
    if len(sys.argv)>1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(1, "lo")

    ddsHandler = DDSHandler(LR='r')
    # ddsHandler = DDSHandler(LR='l')

    while True:
        time.sleep(1)
