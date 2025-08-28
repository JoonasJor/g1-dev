import time
import numpy as np
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from controller import ArmController, HandController
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from inspire.modbus_data_handler import ModbusDataHandler

import config as Cfg
import g1_joints as Joints

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

def test_loop(arms: ArmController):
    while arms.low_state is None:
        print(f"Waiting for low_state, {arms.low_state=}")
        time.sleep(1)
    time.sleep(2)

    while True:
        print("Moving arms 60...")
        angles = [0] * 29
        angles[Joints.Body.LeftShoulderRoll.idx] = np.deg2rad(60)
        angles[Joints.Body.LeftElbow.idx] = np.deg2rad(60)
        angles[Joints.Body.RightShoulderRoll.idx] = np.deg2rad(-60)
        angles[Joints.Body.RightElbow.idx] = np.deg2rad(60)

        arms.low_cmd_control(
            target_angles=angles,
            kp=arms.kp,
            kd=arms.kd,
            duration=2.0
        )

        print("Moving arms 30...")
        angles = [0] * 29
        angles[Joints.Body.LeftShoulderRoll.idx] = np.deg2rad(30)
        angles[Joints.Body.LeftElbow.idx] = np.deg2rad(30)
        angles[Joints.Body.RightShoulderRoll.idx] = np.deg2rad(-30)
        angles[Joints.Body.RightElbow.idx] = np.deg2rad(30)

        arms.low_cmd_control(
            target_angles=angles,
            kp=arms.kp,
            kd=arms.kd,
            duration=2.0
        )

def arms_default(arms: ArmController):
    while arms.low_state is None:
        print(f"Waiting for low_state, {arms.low_state=}")
        time.sleep(1)
    time.sleep(2)

    print("Moving arms to default position...")
    angles = [0] * 29
    angles[Joints.Body.LeftElbow.idx] = np.deg2rad(30)
    angles[Joints.Body.RightElbow.idx] = np.deg2rad(30)  
    arms.low_cmd_control(
        target_angles=angles,
        kp=arms.kp,
        kd=arms.kd,
        duration=3.0
    )

def test_cart_push():
    arms = ArmController()
    hand_r = HandController("r")
    hand_l = HandController("l")

    while arms.low_state is None:
        print(f"Waiting for low_state, {arms.low_state=}")
        time.sleep(1)
    time.sleep(2)

    print("Moving arms to zero position...")
    angles = [0] * 29
    arms.low_cmd_control(
        target_angles=angles,
        kp=arms.kp,
        kd=arms.kd,
        duration=3.0
    )

    print("Moving arms to position...")
    angles = [0] * 29
    angles[Joints.Body.LeftElbow.idx] = np.deg2rad(-60)
    angles[Joints.Body.RightElbow.idx] = np.deg2rad(-60)  

    #angles[Joints.Body.LeftShoulderPitch.idx] = np.deg2rad(45)
    #angles[Joints.Body.RightShoulderPitch.idx] = np.deg2rad(-45)

    angles[Joints.Body.LeftWristRoll.idx] = np.deg2rad(90)
    angles[Joints.Body.RightWristRoll.idx] = np.deg2rad(-90)

    arms.low_cmd_control(
        target_angles=angles,
        kp=arms.kp,
        kd=arms.kd,
        duration=2.0
    )

    #return

    print("Opening hands...")
    open_hand(hand_r)
    open_hand(hand_l)

    print("Closing hands...")
    close_hand(hand_r)
    close_hand(hand_l)


def test(arms: ArmController):
    while arms.low_state is None:
        print(f"Waiting for low_state, {arms.low_state=}")
        time.sleep(1)
    time.sleep(2)

    while True:
        print("Moving arms...")
        angles = [0] * 29
        angles[Joints.Body.LeftShoulderRoll.idx] = np.deg2rad(90)
        angles[Joints.Body.LeftElbow.idx] = np.deg2rad(90)
        angles[Joints.Body.RightShoulderRoll.idx] = np.deg2rad(-90)
        angles[Joints.Body.RightElbow.idx] = np.deg2rad(90)

        arms.low_cmd_control(
            target_angles=angles,
            kp=arms.kp,
            kd=arms.kd,
            duration=5.0
        )

        print("Moving arms to zero position...")
        angles = [0] * 29

        arms.low_cmd_control(
            target_angles=angles,
            kp=arms.kp,
            kd=arms.kd,
            duration=5.0
        )  

        #print("Releasing arm sdk control...")
        #arm_controller.release_control()  

if __name__ == '__main__':
    if len(sys.argv) > 1:
        # Run on real robot
        ChannelFactoryInitialize(0, sys.argv[1])
        #modbus_r = ModbusDataHandler(ip="192.168.123.211", device_id=1, l_r="r")
        #modbus_l = ModbusDataHandler(ip="192.168.123.210", device_id=2, l_r="l")
    else:
        # Run in Mujoco
        ChannelFactoryInitialize(1, "lo")

    arm_controller = ArmController()
    #arms_default(arm_controller)

    #test(arm_controller)
    test_loop(arm_controller)

    #test_cart_push()
