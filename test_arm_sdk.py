import time
import numpy as np
import sys

from controller import ArmController
from unitree_sdk2py.core.channel import ChannelFactoryInitialize

import config as Cfg
import g1_joints as Joints

def test(arms: ArmController):
    while arms.low_state is None:
        print(f"[Demo] Waiting for low_state, {arms.low_state=}")
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

if __name__ == '__main__':
    if len(sys.argv) > 1:
        # Run on real robot
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        # Run in Mujoco
        ChannelFactoryInitialize(1, "lo")

    arm_controller = ArmController()
    test(arm_controller)