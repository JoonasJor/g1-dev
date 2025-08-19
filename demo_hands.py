import time
import numpy as np
import sys

from controller import HandController
from unitree_sdk2py.core.channel import ChannelFactoryInitialize

import config as Cfg
import g1_joints as Joints

def pinch(hand: HandController):
    print("Opening hand...")
    angles = [0] * 6
    hand.low_cmd_control(
        target_angles=angles,
        duration=3.0
    )

    print("Pinching with index and thumb...")
    # Pinky, Ring, Middle, Index, Thumb-bend, Thumb-rotation
    # 0 - 1000
    angles = [0, 0, 0, 700, 750, 1000]
    hand.low_cmd_control(
        target_angles=angles,
        duration=3.0,
        wait_for_user_input=False
    )

    return

    print("Opening hand...")
    angles = [0] * 6
    hand.low_cmd_control(
        target_angles=angles,
        duration=3.0
    )

def hands_loop(hand_r: HandController, hand_l: HandController):
    while True:
        print("Closing right hand...")
        angles = [1000] * 6
        hand_r.low_cmd_control(
            target_angles=angles,
            duration=2.0,
            wait_for_user_input=False
        )

        print("Closing left hand...")
        angles = [1000] * 6
        hand_l.low_cmd_control(
            target_angles=angles,
            duration=2.0,
            wait_for_user_input=False
        )

        print("Opening right hand...")
        angles = [0] * 6
        hand_r.low_cmd_control(
            target_angles=angles,
            duration=2.0,
            wait_for_user_input=False
        )

        print("Opening left hand...")
        angles = [0] * 6
        hand_l.low_cmd_control(
            target_angles=angles,
            duration=2.0,
            wait_for_user_input=False
        )

if __name__ == '__main__':
    if len(sys.argv) > 1:
        # Run on real robot
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        # Run in Mujoco
        ChannelFactoryInitialize(1, "lo")
        
    hand_controller_r = HandController("r")
    hand_controller_l = HandController("l")

    #hands_loop(hand_controller_r, hand_controller_l)

    pinch(hand_controller_r)
    pinch(hand_controller_l)