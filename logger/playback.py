import os 
import sys
import threading

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_, MotorState_

from controller import BodyController, ArmController, HandController
import g1_joints as Joints

import inspire.inspire_dds as inspire_dds
import camera.camera_dds as camera_dds

from logger.g1_logger import G1_Logger

if __name__ == "__main__":
    if len(sys.argv) > 1:
        # Run on real robot
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        # Run in Mujoco
        ChannelFactoryInitialize(1, "lo")

    body = BodyController(flg_initialized=True)
    arms = ArmController()
    hand_r = HandController(l_r="r")
    hand_l = HandController(l_r="l")

    data_list = G1_Logger.load_data_from_file("log/data_latest.pkl")
    
    for i, data in enumerate(data_list):
        timestamp = data_list[i]["timestamp"]
        timestamp_next = data_list[i + 1]["timestamp"] if i + 1 < len(data_list) else None

        body_low_state: LowState_ = data["body"]
        body_joint_angles = [motor.q for motor in body_low_state.motor_state]
        body_joint_torques = [motor.tau_est for motor in body_low_state.motor_state]
        body_joint_velocities = [motor.dq for motor in body_low_state.motor_state]
        body_joint_kp = [motor.kp for motor in body_low_state.motor_state]
        body_joint_kd = [motor.kd for motor in body_low_state.motor_state]

        hand_r_state: inspire_dds.inspire_hand_state = data["hand_r"]["state"]
        hand_r_joint_angles = hand_r_state.angle_act

        hand_l_state: inspire_dds.inspire_hand_state = data["hand_l"]["state"]
        hand_l_joint_angles = hand_l_state.angle_act

        camera_data: camera_dds.camera_image = data["camera"]

        timestamp_delta = timestamp_next - timestamp

        ankle_motor_state: MotorState_ = body_low_state.motor_state[Joints.Body.LeftAnklePitch.idx]
        print(f"{ankle_motor_state.tau_est=}")

        if i == 0:
            print("Moving to initial position...")
            hand_r.low_cmd_control(hand_r_joint_angles, duration=2.0)
            hand_l.low_cmd_control(hand_l_joint_angles, duration=2.0)
            body.low_cmd_control(body_joint_angles, target_torques=body_joint_torques, duration=2.0)
            body.repeat_last_command()
            continue

        if timestamp_next is not None:
            threads = [
                threading.Thread(target=body.low_cmd_control, args=(body_joint_angles,), 
                                 kwargs={"target_torques": body_joint_torques, 
                                         "target_velocities": body_joint_velocities,
                                         "kp": body_joint_kp,
                                         "kd": body_joint_kd,
                                         "duration": timestamp_delta, 
                                         "interpolate": False}),
                threading.Thread(target=hand_r.low_cmd_control, args=(hand_r_joint_angles,), kwargs={"duration": timestamp_delta}),
                threading.Thread(target=hand_l.low_cmd_control, args=(hand_l_joint_angles,), kwargs={"duration": timestamp_delta}),
            ]

            for thread in threads:
                thread.start()
            for thread in threads:
                thread.join()

