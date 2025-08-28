import time
import numpy as np
import sys

from controller import HandController
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from inspire.modbus_data_handler import ModbusDataHandler

import config as Cfg
import g1_joints as Joints

# Index =   0       1       2       3       4           5
# Joint =   Pinky   Ring    Middle  Index   Thumb-bend  Thumb-rotation
# Range =   0-1000

def open_hand(hand: HandController):
    angles = [1000] * 6
    hand.low_cmd_control(
        target_angles=angles,
        duration=1.0
    )

def close_hand(hand: HandController):
    angles = [0, 0, 0, 0, 0, 500]
    hand.low_cmd_control(
        target_angles=angles,
        duration=1.0
    )

def pinch(hand: HandController):
    print("Pinching with index and thumb...")
    angles = [1000, 1000, 1000, 300, 300, 0]
    hand.low_cmd_control(
        target_angles=angles,
        duration=3.0
    )

def hands_loop(hand_r: HandController, hand_l: HandController):
    while True:
        print("Closing right hand...")
        close_hand(hand_r)

        print("Closing left hand...")
        close_hand(hand_l)

        print("Opening right hand...")
        open_hand(hand_r)

        print("Opening left hand...")
        open_hand(hand_l)

if __name__ == '__main__':
    if len(sys.argv) > 1:
        # Run on real robot
        ChannelFactoryInitialize(0, sys.argv[1])
        modbus_r = ModbusDataHandler(ip="192.168.123.211", device_id=1, l_r="r")
        modbus_l = ModbusDataHandler(ip="192.168.123.210", device_id=2, l_r="l")
    else:
        # Run in Mujoco
        ChannelFactoryInitialize(1, "lo")

    hand_controller_r = HandController(l_r="r")
    hand_controller_l = HandController(l_r="l")
        
    #close_hand(hand_controller_l)
    #close_hand(hand_controller_r)

    #open_hand(hand_controller_r)
    #open_hand(hand_controller_l)

    #while True: time.sleep(1)

    hands_loop(hand_controller_r, hand_controller_l)

    #pinch(hand_controller_r)
    #pinch(hand_controller_l)