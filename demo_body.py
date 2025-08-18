import time
import numpy as np
import sys

from controller import BodyController
from unitree_sdk2py.core.channel import ChannelFactoryInitialize

import config as Cfg
import g1_joints as Joints

def demo_floor(body: BodyController):
    while body.low_state is None:
        print(f"[Demo] Waiting for low_state, {body.low_state=}")
        time.sleep(1)
    time.sleep(2)

    start_angles = [motor.q for motor in body.low_state.motor_state]

    print("Setting joints to zero position...")
    angles = [0] * 29
    body.low_cmd_control(
        target_angles=angles,
        kp=body.kp,
        kd=body.kd,
        duration=6.0
    )

    print("Moving right leg...")
    angles = [motor.q for motor in body.low_state.motor_state]
    angles[Joints.Body.RightHipRoll.idx] = np.deg2rad(45)
    angles[Joints.Body.RightKnee.idx] = np.deg2rad(90)
    angles[Joints.Body.RightAnklePitch.idx] = np.deg2rad(90)

    body.low_cmd_control(
        target_angles=angles,
        kp=body.kp,
        kd=body.kd,
        duration=6.0
    )

    print("Setting joints to start position...")
    body.low_cmd_control(
        target_angles=start_angles,
        kp=body.kp,
        kd=body.kd,
        duration=6.0
    )

def demo_stand(body: BodyController):
    while body.low_state is None:
        print(f"[Demo] Waiting for low_state, {body.low_state=}")
        time.sleep(1)
    time.sleep(2)

    start_angles = [motor.q for motor in body.low_state.motor_state]

    print("Setting body joints to default position...")
    body.low_cmd_control(
        target_angles=Joints.Body.default_angles_list(),
        kp=body.kp,
        kd=body.kd,
        duration=6.0
    )

    print("Moving arm...")
    angles = [motor.q for motor in body.low_state.motor_state]
    angles[Joints.Body.LeftElbow.idx] = np.deg2rad(-45)
    body.low_cmd_control(
        target_angles=angles,
        kp=body.kp,
        kd=body.kd,
        duration=6.0
    )

    print("Setting body joints to start position...")
    body.low_cmd_control(
        target_angles=start_angles,
        kp=body.kp,
        kd=body.kd,
        duration=6.0
    )

if __name__ == '__main__':
    if len(sys.argv) > 1:
        # Run on real robot
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        # Run in Mujoco
        ChannelFactoryInitialize(1, "lo")

    body_controller = BodyController()
    body_controller.init_msc()

    if Cfg.START_ON_FLOOR:
        demo_floor(body_controller)
    else:
        demo_stand(body_controller)